#include "roboteq_nxtgen_controller/nxtgen_driver.h"

#include <cmath>
#include <sstream>

namespace nxtgen_driver
{

NxtGenDriver::NxtGenDriver( ros::NodeHandle &nh ) :
	nh( nh ),
	updater( ),
	running( false ),
	error_count( 0 )
{
	ros::NodeHandle nh_priv( "~" );

	nh_priv.param<std::string>( "hardware_id", hardware_id, "RoboteQ NxtGen" );
	nh_priv.param<std::string>( "port", port, "/dev/ttyUSB0" );
	nh_priv.param<bool>( "single_channel", single_channel, true );
	nh_priv.param<bool>( "invert", invert, false );

	nh_priv.param<bool>( "enable_watchdog", enable_watchdog, true );
	nh_priv.param<double>( "watchdog_timeout", watchdog_timeout, 1 );

	// Valid watchdog timeout is 0 to 65 seconds
	if( watchdog_timeout <= 0 )
	{
		enable_watchdog = false;
		watchdog_timeout = 0;
	}
	else if( watchdog_timeout > 65 )
		watchdog_timeout = 65;

	nh_priv.param<bool>( "use_encoders", use_encoders, false );
	nh_priv.param<int>( "encoder_type", encoder_type, 0 );
	nh_priv.param<int>( "encoder_ppr", encoder_ppr, 200 );
	nh_priv.param<bool>( "reset_encoder_count", reset_encoder_count, false );

	std::string mode;
	nh_priv.param<std::string>( "operating_mode", mode, "open-loop speed" );

	if( mode == "open-loop speed" )
		operating_mode = OperatingModes::OpenLoopSpeed;
	else if( mode == "closed-loop speed" )
		operating_mode = OperatingModes::ClosedLoopSpeed;
	else if( mode == "closed-loop position" )
		operating_mode = OperatingModes::ClosedLoopPosition;
	else
	{
		ROS_WARN( "Invalid operating mode [%s]. Valid operating modes are " \
			"'open-loop speed', 'closed-loop speed', or 'closed-loop position' " \
			"-- defaulting to 'open-loop speed'", mode.c_str( ) );
		operating_mode = OperatingModes::OpenLoopSpeed;
	}

	nh_priv.param<int>( "ch1_max_motor_rpm", ch1_max_motor_rpm, 3000 );
	nh_priv.param<int>( "ch2_max_motor_rpm", ch2_max_motor_rpm, 3000 );

	nh_priv.param<std::string>( "ch1_joint_name", ch1_joint_name, "channel1" );
	nh_priv.param<std::string>( "ch2_joint_name", ch2_joint_name, "channel2" );

	nh_priv.param<bool>( "publish_joint_states", publish_joint_states, true );
	nh_priv.param<double>( "joint_states_publish_rate", joint_states_publish_rate, 30.0 );

	if( joint_states_publish_rate <= 0.0 )
	{
		expected_joint_states_publish_time = 0.0;
		publish_joint_states = false;
	}
	else
		expected_joint_states_publish_time = ( 1.0 / joint_states_publish_rate );

	joint_traj_sub = nh.subscribe( "cmd_joint_traj", 1, &NxtGenDriver::jointTrajCallback, this );
	joint_state_pub = nh.advertise<sensor_msgs::JointState>( "joint_states", 1 );
	reset_enc_srv = nh_priv.advertiseService( "reset_encoder_count", &NxtGenDriver::resetEncoderCallback, this );

	updater.setHardwareID( hardware_id );
	updater.add( hardware_id + " Status", this, &NxtGenDriver::deviceStatus );

	dyn_recog_type = boost::bind( &NxtGenDriver::dynRecogCallback, this, _1, _2 );
}

NxtGenDriver::~NxtGenDriver( )
{
	stop( );

	joint_traj_sub.shutdown( );
	joint_state_pub.shutdown( );
}

bool NxtGenDriver::spin( )
{
	if( !start( ) )
		return false;

	if( !init( ) )
	{
		ROS_ERROR( "Unable to initialize device" );
		return false;
	}

	dyn_recog_server.setCallback( dyn_recog_type );

	while( nh.ok( ) )
	{
		update( );

		updater.update( );

		ros::spinOnce( );
	}

	dyn_recog_server.clearCallback( );

	return true;
}

bool NxtGenDriver::init( )
{
	int result;

	// Set channel 1 operating mode
	result = dev.SetConfig( _MMOD, 1, operating_mode );
	if( !checkResult( result ) )
		return false;

	if( !single_channel )
	{
		// Set channel 2 operating mode
		result = dev.SetConfig( _MMOD, 2, operating_mode );
		if( !checkResult( result ) )
			return false;
	}

	// Set RS232 watchdog timeout (0 means disabled)
	result = dev.SetConfig( _RWD, enable_watchdog ? ( 1000 * ( int )watchdog_timeout ) : 0 );
	if( !checkResult( result ) )
		return false;

	// Set encoder 1 usage
	result = dev.SetConfig( _EMOD, 1, use_encoders ? 17 : 16 );
	if( !checkResult( result ) )
		return false;

	if( !single_channel )
	{
		// Set encoder 2 usage
		result = dev.SetConfig( _EMOD, 2, use_encoders ? 33 : 32 );
		if( !checkResult( result ) )
			return false;
	}

	// Encoder type (0 = hall sensor, any other value = optical encoder)
	result = dev.SetConfig( _BLFB, ( encoder_type == 0 ) ? 0 : 1 );

	if( reset_encoder_count )
		if( !resetEncoderCount( ) )
			return false;

	// Set motor 1 max rpm
	result = dev.SetConfig( _MXRPM, 1, ch1_max_motor_rpm );
	if( !checkResult( result ) )
		return false;

	if( !single_channel )
	{
		// Set motor 2 max rpm
		result = dev.SetConfig( _MXRPM, 2, ch2_max_motor_rpm );
		if( !checkResult( result ) )
			return false;
	}

	// Set encoder 1 PPR
	result = dev.SetConfig( _EPPR, 1, encoder_ppr );
	if( !checkResult( result ) )
		return false;

	if( !single_channel )
	{
		// Set encoder 2 PPR
		result = dev.SetConfig( _EPPR, 2, encoder_ppr );
		if( !checkResult( result ) )
			return false;
	}

	return true;
}

bool NxtGenDriver::start( )
{
	int result = -1;

	ROS_INFO( "Starting..." );
	ROS_INFO( "Attempting to open device" );

	for( int i = 0; i < 3 && result != RQ_SUCCESS; i++ )
	{
		result = dev.Connect( port );

		if( result != RQ_SUCCESS )
		{
			ROS_WARN( "Attempt number %i of 3 failed.", i + 1 );

			sleep( 1 );

			if( i < 2 )
				ROS_WARN( "Trying again..." );
		}
	}

	if( !checkResult( result ) )
		return false;

	ROS_INFO( "Succesfully opened device %s", port.c_str( ) );

	running = true;

	return true;
}

void NxtGenDriver::stop( )
{
	ROS_INFO( "Stopping..." );

	dev.Disconnect( );

	running = false;
}

void NxtGenDriver::update( )
{
	current_time = ros::Time::now( );

	if( publish_joint_states )
		publishJointStates( );

	last_time = current_time;
}

void NxtGenDriver::publishJointStates( )
{
	if( fabs( ( last_joint_states_publish_time - current_time ).toSec( ) ) >= expected_joint_states_publish_time )
	{
		int ch1_pos, ch2_pos;
		int ch1_vel, ch2_vel;

		if( getMotorRPM( ch1_vel, ch2_vel ) && getEncoderCountAbs( ch1_pos, ch2_pos ) )
		{
			if( invert )
			{
				ch1_pos *= -1.0;
				ch2_pos *= -1.0;
				ch1_vel *= -1.0;
				ch2_vel *= -1.0;
			}

			sensor_msgs::JointState joints;
			joints.header.stamp = current_time;
			//joints.header.frame_id = joint_state_frame;

			joints.name.push_back( ch1_joint_name );
			joints.position.push_back( 2.0 * M_PI * ( double )ch1_pos / ( 4.0 * ( double )encoder_ppr ) );
			joints.velocity.push_back( 2.0 * M_PI * ( double )ch1_vel / 60.0 );

			if( !single_channel )
			{
				joints.name.push_back( ch2_joint_name );
				joints.position.push_back( 2.0 * M_PI * ( double )ch2_pos / ( 4.0 * ( double )encoder_ppr ) );
				joints.velocity.push_back( 2.0 * M_PI * ( double )ch2_vel / 60.0 );
			}

			joint_state_pub.publish( joints );

			last_joint_states_publish_time = current_time;
		}
	}
}

bool NxtGenDriver::getDigitalInputConfig( DigitalInputConfig &config )
{
	int action;
	int level;
	int result;
	int res_amount = 8;

	/// \todo Find a way to get the controller's model

	//if( model == HDC2450 ) // The HDC2450 has 20 digital inputs
	//	res_amount = 20;

	res_amount = 6;

	config.inputs.reserve( res_amount );
	config.levels.reserve( res_amount );
	config.motor1.reserve( res_amount );
	config.motor2.reserve( res_amount );

	for( int i = 0; i < res_amount; i++ )
	{
		// Get action for digital input i
		result = dev.GetConfig( _DINA, i + 1, action );
		if( !checkResult( result ) )
			return false;

		// Get active level
		result = dev.GetConfig( _DINL, i + 1, level );
		if( !checkResult( result ) )
			return false;

		config.inputs.push_back( DigitalInputAction( action & 15 ) ); // Only first four bits are the action
		config.levels.push_back( DigitalActionLevel( level ) );
		config.motor1.push_back( bool( action & 32 ) ); // bit 5 gives motor 1
		config.motor2.push_back( bool( action & 64 ) ); // bit 6 gives motor 2
	}

	return true;
}

bool NxtGenDriver::getDigitalOutputConfig( DigitalOutputConfig &config )
{
	int action;
	int level;
	int result;
	int res_amount = 2;

	/// \todo Find a way to get the controller's model

	//if( model == HDC2450 ) // The HDC2450 has 8 digital outputs
	//	res_amount = 8;

	config.outputs.reserve( res_amount );
	config.levels.reserve( res_amount );
	config.motor1.reserve( res_amount );
	config.motor2.reserve( res_amount );

	for( int i = 0; i < res_amount; i++ )
	{
		// Get action for digital output i
		result = dev.GetConfig( _DOA, i + 1, action );
		if( !checkResult( result ) )
			return false;

		// Get active level
		result = dev.GetConfig( _DOL, i + 1, level );
		if( !checkResult( result ) )
			return false;

		config.outputs.push_back( DigitalOutputAction( action & 7 ) ); // First 3 bits give the action
		config.levels.push_back( DigitalActionLevel( level ) );
		config.motor1.push_back( bool( action & 16 ) ); // bit 4 gives motor 1
		config.motor2.push_back( bool( action & 32 ) ); // bit 5 gives motor 2
	}

	return true;
}

bool NxtGenDriver::getEncoderCountAbs( int &enc1, int &enc2 )
{
	int result;

	result = dev.GetValue( _ABCNTR, 1, enc1 );
	if( !checkResult( result ) )
		return false;

	if( !single_channel )
	{
		result = dev.GetValue( _ABCNTR, 2, enc2 );
		if( !checkResult( result ) )
			return false;
	}

	return true;
}

bool NxtGenDriver::getEncoderCountRel( int &enc1, int &enc2 )
{
	int result;

	result = dev.GetValue( _RELCNTR, 1, enc1 );
	if( !checkResult( result ) )
		return false;

	if( !single_channel )
	{
		result = dev.GetValue( _RELCNTR, 2, enc2 );
		if( !checkResult( result ) )
			return false;
	}

	return true;
}

bool NxtGenDriver::getMotorRPM( int &ch1, int &ch2 )
{
	int result;

	result = dev.GetValue( _ABSPEED, 1, ch1 );
	if( !checkResult( result ) )
		return false;

	if( !single_channel )
	{
		result = dev.GetValue( _ABSPEED, 2, ch2 );
		if( !checkResult( result ) )
			return false;
	}

	return true;
}

std::string NxtGenDriver::operatingModeToStr( OperatingMode op_mode )
{
	switch( op_mode )
	{
		case OperatingModes::OpenLoopSpeed: 		return "Open Loop Speed";
		case OperatingModes::ClosedLoopSpeed:		return "Closed Loop Speed";
		case OperatingModes::ClosedLoopPosition: 	return "Closed Loop Position";
		default:					return "Unknown";
	}
}

std::string NxtGenDriver::digitalActionLevelToStr( DigitalActionLevel level )
{
	switch( level )
	{
		case DigitalActionLevels::ActiveHigh:	return "Active high";
		case DigitalActionLevels::ActiveLow:	return "Active low";
		default:				return "Unknown";
	}
}

std::string NxtGenDriver::digitalInputActionToStr( DigitalInputAction action )
{
	switch( action )
	{
		case DigitalInputActions::NoAction:		return "No action";
		case DigitalInputActions::SafetyStop:		return "Safety stop";
		case DigitalInputActions::EmergencyStop:	return "Emergency stop";
		case DigitalInputActions::MotorStop:		return "Motor stop";
		case DigitalInputActions::ForwardLimitSwitch:	return "Forward limit switch";
		case DigitalInputActions::ReverseLimitSwitch:	return "Reverse limit switch";
		case DigitalInputActions::InvertDirection:	return "Invert direction";
		case DigitalInputActions::RunMicroBasicScript:	return "Run MicroBasic script";
		case DigitalInputActions::LoadHomeCounterValue:	return "Load counter with home value";
		default:					return "Unknown";
	}
}

std::string NxtGenDriver::digitalOutputActionToStr( DigitalOutputAction action )
{
	switch( action )
	{
		case DigitalOutputActions::NoAction:		return "No action";
		case DigitalOutputActions::WhenMotorOn:		return "When motor on";
		case DigitalOutputActions::MotorReversed:	return "Motor reversed";
		case DigitalOutputActions::Overvoltage:		return "Overvoltage";
		case DigitalOutputActions::Overtemperature:	return "Overtemperature";
		case DigitalOutputActions::MirrorStatusLED:	return "Mirror status LED";
		case DigitalOutputActions::NoMOSFETFailure:	return "No MOSFET failure";
		default:					return "Unknown";
	}
}

bool NxtGenDriver::resetEncoderCount( )
{
	int result;
	bool reload_op_mode = false;
	OperatingMode prev_op_mode = operating_mode;

	ROS_DEBUG( "Resetting encoder count..." );

	// It's only safe to reset the encoder count in open-loop mode.
	// Otherwise, the controller may see a huge jump in the encoder count
	// and think the wheels are moving, with the result of it applying
	// power to the motors (which we don't want it to do).
	if( operating_mode != OperatingModes::OpenLoopSpeed )
	{
		ROS_DEBUG( "Temporarily setting controller into open-loop speed mode." );

		if( !setOperatingMode( OperatingModes::OpenLoopSpeed ) )
			return false;

		reload_op_mode = true;
	}

	// Reset encoder 1
	result = dev.SetCommand( _HOME, 1 );
	if( !checkResult( result ) )
		return false;

	if( !single_channel )
	{
		// Reset encoder 2
		result = dev.SetCommand( _HOME, 2 );
		if( !checkResult( result ) )
			return false;
	}

	ROS_DEBUG( "Successfully reset the encoders." );

	if( reload_op_mode )
	{
		ROS_DEBUG( "Reloading previous operating mode" );

		if( !setOperatingMode( prev_op_mode ) )
			return false;

		ROS_DEBUG( "The previous operating mode has been successfully reloaded." );
	}

	return true;
}

bool NxtGenDriver::setOperatingMode( OperatingMode mode )
{
	int result;
	const char* mode_str = operatingModeToStr( mode ).c_str( );

	ROS_DEBUG( "Setting operating mode to %s", mode_str );

	// Set channel 1 operating mode
	result = dev.SetConfig( _MMOD, 1, mode );
	if( !checkResult( result ) )
		return false;

	if( !single_channel )
	{
		// Set channel 2 operating mode
		result = dev.SetConfig( _MMOD, 2, mode );
		if( !checkResult( result ) )
			return false;
	}

	ROS_DEBUG( "Verifing controller is in %s mode.", mode_str );

	// Get channel 1's operating mode
	result = dev.GetConfig( _MMOD, 1, *( ( int* )&operating_mode ) );
	if( !checkResult( result ) )
		return false;

	// Verify channel 1 is in the correct mode
	if( operating_mode != mode )
	{
		ROS_ERROR( "Failed to set channel 1 to %s mode.", mode_str );
		error_count++;
		return false;
	}

	if( !single_channel )
	{
		// Get channel 2's operating mode
		result = dev.GetConfig( _MMOD, 2, *( ( int* )&operating_mode ) );
		if( !checkResult( result ) )
			return false;

		// Verify channel 2 is in open-loop speed mode
		if( operating_mode != mode )
		{
			ROS_ERROR( "Failed to set channel 2 to %s mode.", mode_str );
			error_count++;
			return false;
		}
	}

	return true;
}

void NxtGenDriver::jointTrajCallback( const trajectory_msgs::JointTrajectoryConstPtr &msg )
{
	float rpm;
	int result;

	for( unsigned int i = 0; i < msg->joint_names.size( ); i++ )
	{
		if( msg->joint_names[i] == ch1_joint_name )
		{
			if( operating_mode == OperatingModes::OpenLoopSpeed )
				rpm = 1000 * msg->points[0].velocities[i];
			else
			{
				rpm = 60.0 * msg->points[0].velocities[i] / ( 2.0 * M_PI );

				// Calculate rpm percentate for closed loop mode
				rpm = 1000.0 * rpm / ( float )ch1_max_motor_rpm;
			}

			/// \todo Support closed-loop position mode

			if( invert ) rpm *= -1;

			// Round rpm value and send to channel 1
			result = dev.SetCommand( _GO, 1, ( int )( rpm + 0.5 ) );

			if( !checkResult( result) )
			{
				ROS_ERROR( "Sending command to %s failed", ch1_joint_name.c_str( ) );
			}
		}
		else if( msg->joint_names[i] == ch2_joint_name )
		{
			if( operating_mode == OperatingModes::OpenLoopSpeed )
				rpm = 1000 * msg->points[0].velocities[i];
			else
			{
				rpm = 60.0 * msg->points[0].velocities[i] / ( 2.0 * M_PI );

				// Calculate rpm percentage for closed loop mode
							rpm = 1000.0 * rpm / ( float )ch2_max_motor_rpm;
			}

			/// \todo Support closed-loop position mode

			if( invert ) rpm *= -1;

			// Round rpm value and send to channel 2
			result = dev.SetCommand( _GO, 2, ( int )( rpm + 0.5 ) );

			// Verify the command was successfully sent
			if( !checkResult( result ) )
			{
				ROS_ERROR( "Sending command to %s failed", ch2_joint_name.c_str( ) );
			}
		}
	}
}

void NxtGenDriver::deviceStatus( diagnostic_updater::DiagnosticStatusWrapper &status )
{
	int ch1_op_mode;
	int ch2_op_mode;
	int driver_voltage;
	int main_battery_voltage;
	int dsub_5v_connector_voltage;
	int battery1_amps;
	int battery2_amps;
	int motor1_amps;
	int motor2_amps;
	int internal_ic_temp;
	int heatsink_temp;
	int motor1_power_output;
	int motor2_power_output;
	int fault_flag;
	DigitalInputConfig din_config;
	DigitalOutputConfig dout_config;

	int result;
	bool errors = false;
	stringstream ss;

	if( running )
	{
		result = dev.GetValue( _MMOD, 1, ch1_op_mode );
		if( !checkResult( result ) )
			errors = true;

		if( !single_channel )
		{
			result = dev.GetValue( _MMOD, 2, ch2_op_mode );
			if( !checkResult( result ) )
				errors = true;
		}

		result = dev.GetValue( _VOLTS, 1, driver_voltage );
		if( !checkResult( result ) )
			errors = true;

		result = dev.GetValue( _VOLTS, 2, main_battery_voltage );
		if( !checkResult( result ) )
			errors = true;

		result = dev.GetValue( _VOLTS, 3, dsub_5v_connector_voltage );
		if( !checkResult( result ) )
			errors = true;

		result = dev.GetValue( _BATAMPS, 1, battery1_amps );
		if( !checkResult( result ) )
			errors = true;

		if( !single_channel )
		{
			result = dev.GetValue( _BATAMPS, 2, battery2_amps );
			if( !checkResult( result ) )
				errors = true;
		}

		result = dev.GetValue( _MOTAMPS, 1, motor1_amps );
		if( !checkResult( result ) )
			errors = true;

		if( !single_channel )
		{
			result = dev.GetValue( _MOTAMPS, 2, motor2_amps );
			if( !checkResult( result ) )
				errors = true;
		}

		result = dev.GetValue( _TEMP, 1, internal_ic_temp );
		if( !checkResult( result ) )
			errors = true;

		result = dev.GetValue( _TEMP, 2, heatsink_temp );
		if( !checkResult( result ) )
			errors = true;

		result = dev.GetValue( _MOTPWR, 1, motor1_power_output );
		if( !checkResult( result ) )
			errors = true;

		if( !single_channel )
		{
			result = dev.GetValue( _MOTPWR, 2, motor2_power_output );
			if( !checkResult( result ) )
				errors = true;
		}

		result = dev.GetValue( _FLTFLAG, fault_flag );
		if( !checkResult( result ) )
			errors = true;

		if( !getDigitalInputConfig( din_config ) )
		{
			ROS_DEBUG( "Failed getting digital input configuration" );
			errors = true;
		}

		if( !getDigitalOutputConfig( dout_config ) )
		{
			ROS_DEBUG( "Failed getting digital output configuration" );
			errors = true;
		}

		if( !errors )
			status.summary( diagnostic_msgs::DiagnosticStatus::OK, hardware_id + " is running" );
		else
			status.summary( diagnostic_msgs::DiagnosticStatus::ERROR, "Failed while reading diagnostics" );

		
		status.addf( "Channel 1 Operating Mode", "%s (%d)", operatingModeToStr( OperatingMode( ch1_op_mode ) ).c_str( ), ch1_op_mode );
		status.addf( "Channel 2 Operating Mode", "%s (%d)", operatingModeToStr( OperatingMode( ch2_op_mode ) ).c_str( ), ch2_op_mode );
		status.add( "Main battery voltage", ( double )main_battery_voltage / 10.0 );
		status.add( "Internal voltage", ( double )driver_voltage / 10.0 );
		status.add( "5V DSub connector voltage", ( double )dsub_5v_connector_voltage / 1000.0 );
		status.add( "Motor 1 Amps", ( double )motor1_amps / 10.0 );
		status.add( "Motor 2 Amps", ( double )motor2_amps / 10.0 );
		status.add( "Battery 1 Amps", ( double )battery1_amps / 10.0 );
		status.add( "Battery 2 Amps", ( double )battery2_amps / 10.0 );
		status.add( "Internal IC temperature", internal_ic_temp );
		status.add( "Heatsink temperature", heatsink_temp );
		status.add( "Motor 1 power output", motor1_power_output );
		status.add( "Motor 2 power output", motor2_power_output );
		status.add( "Overheat detected", ( bool )( fault_flag & 1 ) );
		status.add( "Overvoltage detected", ( bool )( fault_flag & 2 ) );
		status.add( "Undervoltage detected", ( bool )( fault_flag & 4 ) );
		status.add( "Short circuit detected", ( bool )( fault_flag & 8 ) );
		status.add( "Emergency stop activated", ( bool )( fault_flag & 16 ) );
		status.add( "Sepex excitation fault", ( bool )( fault_flag & 32 ) );
		status.add( "EEPROM fault", ( bool )( fault_flag & 64 ) );
		status.add( "Configuration fault", ( bool )( fault_flag & 128 ) );
		status.add( "Error Count", error_count );

		// Append digital input actions
		for( unsigned int i = 0; i < din_config.inputs.size( ); i++ )
		{
			ss << (i + 1);

			//std::string action_key = "Digital Input " + ss.str( ) + " Action";
			std::string action_value = digitalInputActionToStr( din_config.inputs[i] );

			if( din_config.motor1[i] && din_config.motor2[i] )
				action_value += " (motor 1 and 2)";
			else if( din_config.motor1[i] )
				action_value += " (motor 1)";
			else if( din_config.motor2[i] )
				action_value += " (motor 2)";

			status.add( "Digital Input " + ss.str( ) + " Action", action_value );
			status.add( "Digital Input " + ss.str( ) + " Level", digitalActionLevelToStr( din_config.levels[i] ) );

			ss.str( "" );
		}

		// Append digital output actions
		for( unsigned int i = 0; i < dout_config.outputs.size( ); i++ )
		{
			ss << (i + 1);

			std::string action_value = digitalOutputActionToStr( dout_config.outputs[i] );

			if( dout_config.motor1[i] && dout_config.motor2[i] )
				action_value += " (motor 1 and 2)";
			else if( dout_config.motor1[i] )
				action_value += " (motor 1)";
			else if( dout_config.motor2[i] )
				action_value += " (motor 2)";

			status.add( "Digital Output " + ss.str( ) + " Action", action_value );
			status.add( "Digital Output " + ss.str( ) + " Level", digitalActionLevelToStr( dout_config.levels[i] ) );

			ss.str( "" );
		}
	}
	else
		status.summary( diagnostic_msgs::DiagnosticStatus::OK, "RoboteQ NxtGen is stopped" );
}

void NxtGenDriver::dynRecogCallback( roboteq_nxtgen_controller::RoboteqNxtGenConfig &config, uint32_t level )
{
	int result;

	// Channel 1 acceleration
	result = dev.SetConfig( _MAC, 1, config.acceleration );
	checkResult( result );

	if( !single_channel )
	{
		// Channel 2 acceleration
		result = dev.SetConfig( _MAC, 2, config.acceleration );
		checkResult( result );
	}

	// Channel 1 deceleration
	result = dev.SetConfig( _MDEC, 1, config.deceleration );
	checkResult( result );

	if( !single_channel )
	{
		// Channel 2 deceleration
		result = dev.SetConfig( _MDEC, 2, config.deceleration );
		checkResult( result );
	}

	// Channel 1 command linearity
	result = dev.SetConfig( _CLIN, 1, config.command_linearity );
	checkResult( result );

	if( !single_channel )
	{
		// Channel 2 command linearity
		result = dev.SetConfig( _CLIN, 2, config.command_linearity );
		checkResult( result );
	}
}

bool NxtGenDriver::resetEncoderCallback( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res )
{
	return resetEncoderCount( );
}

bool NxtGenDriver::checkResult( int result )
{
	switch( result )
	{
		case RQ_SUCCESS: // 0
			return true;

		case RQ_ERR_OPEN_PORT: // 1
			ROS_ERROR( "An error occured while attempting to open the port" );
			break;

		case RQ_ERR_NOT_CONNECTED: // 2
			ROS_ERROR( "The device is not connected" );
			break;

		case RQ_ERR_TRANSMIT_FAILED: // 3
			ROS_ERROR( "An error occured while transmitting data to device" );
			break;

		case RQ_ERR_SERIAL_IO: // 4
			ROS_ERROR( "An error occured with the serial communication" );
			break;

		case RQ_ERR_SERIAL_RECEIVE: // 5
			ROS_ERROR( "An error occured while receiving data from device" );
			break;

		case RQ_INVALID_RESPONSE: // 6
			ROS_ERROR( "Invalid response to the issued command" );
			break;

		case RQ_UNRECOGNIZED_DEVICE: // 7
			ROS_ERROR( "The device was not recognized" );
			break;

		case RQ_UNRECOGNIZED_VERSION: // 8
			ROS_ERROR( "Invalid device version" );
			break;

		case RQ_INVALID_CONFIG_ITEM: // 9
			ROS_ERROR( "Invalid configuration item, it should be in the range [0,255]" );
			break;

		case RQ_INVALID_OPER_ITEM: // 10
			ROS_ERROR( "Invalid operating item, it should be in the range [0,255]" );
			break;

		case RQ_INVALID_COMMAND_ITEM: // 11
			ROS_ERROR( "Invalid operating item, it should be in the range [0,255]" );
			break;

		case RQ_INDEX_OUT_RANGE: // 12
			ROS_ERROR( "The item index is out of range" );
			break;

		case RQ_SET_CONFIG_FAILED: // 13
			ROS_ERROR( "Failed to set device configuration" );
			break;

		case RQ_GET_CONFIG_FAILED: // 14
			ROS_ERROR( "Failed to get device configuration" );
			break;

		case RQ_GET_VALUE_FAILED: // 15
			ROS_ERROR( "Failed to get operating item configuration" );
			break;

		case RQ_SET_COMMAND_FAILED: // 16
			ROS_ERROR( "Failed to set device command" );
			break;

		case RQ_ERR_SERIAL_TIMEOUT: // 20
			ROS_ERROR( "Timeout will reading from device." );
			break;

		default:
			ROS_ERROR( "An unknown error code was returned" );
			break;
	}

	error_count++;

	return false;
}

} // namespace
