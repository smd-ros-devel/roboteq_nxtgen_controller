#include "roboteq_nxtgen_controller/nxtgen_driver.h"

#include <cmath>

namespace nxtgen_driver
{

NxtGenDriver::NxtGenDriver( ros::NodeHandle &nh ) :
	nh( nh ),
	updater( ),
	running( false )
{
	ros::NodeHandle nh_priv( "~" );

	nh_priv.param<std::string>( "hardware_id", hardware_id, "RoboteQ NxtGen" );
	nh_priv.param<std::string>( "port", port, "/dev/ttyUSB0" );
	nh_priv.param<bool>( "invert", invert, false );
	nh_priv.param<bool>( "use_encoders", use_encoders, false );
	nh_priv.param<int>( "encoder_type", encoder_type, 0 );
	nh_priv.param<int>( "encoder_ppr", encoder_ppr, 200 );

	nh_priv.param<int>( "operating_mode", operating_mode, 1 );

	if( operating_mode < OPEN_LOOP_SPEED || operating_mode > CLOSED_LOOP_POSITION )
	{
		ROS_WARN( "Invalid operating mode [%d]. Operating mode must be 1 (open-loop speed), 2 (closed-loop speed), or 3 (closed-loop) position -- defaulting to 1.", operating_mode );
		operating_mode = 1;
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

	// Set channel 1 operating mode, 1 = open-looped speed,
	// 2 = closed-loop speed, 3 = closed-loop position
	result = dev.SetConfig( _MMOD, 1, operating_mode );
	if( !checkResult( result ) )
		return false;

	// Set channel 2 operating mode, 1 = open-looped speed,
	// 2 = closed-loop speed, 3 = closed-loop position
	result = dev.SetConfig( _MMOD, 2, operating_mode );
	if( !checkResult( result ) )
		return false;

	// Set encoder 1 usage
	result = dev.SetConfig( _EMOD, 1, use_encoders ? 17 : 16 );
	if( !checkResult( result ) )
		return false;

	// Set encoder 2 usage
	result = dev.SetConfig( _EMOD, 2, use_encoders ? 33 : 32 );
	if( !checkResult( result ) )
		return false;

	// Encoder type (0 = hall sensor, any other value = optical encoder)
	result = dev.SetConfig( _BLFB, ( encoder_type == 0 ) ? 0 : 1 );

	// Set motor 1 max rpm
	result = dev.SetConfig( _MXRPM, 1, ch1_max_motor_rpm );
	if( !checkResult( result ) )
		return false;

	// Set motor 2 max rpm
	result = dev.SetConfig( _MXRPM, 2, ch2_max_motor_rpm );
	if( !checkResult( result ) )
		return false;

	// Set encoder 1 PPR
	result = dev.SetConfig( _EPPR, 1, encoder_ppr );
	if( !checkResult( result ) )
		return false;

	// Set encoder 2 PPR
	result = dev.SetConfig( _EPPR, 2, encoder_ppr );
	if( !checkResult( result ) )
		return false;

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

	if( result == RQ_SUCCESS )
	{
		ROS_INFO( "Succesfully opened device %s", port.c_str( ) );
		running = true;
		return true;
	}

	if( result == RQ_ERR_OPEN_PORT )
		ROS_ERROR( "Unable to open device on port %s", port.c_str( ) );
	else if( result == RQ_UNRECOGNIZED_DEVICE )
		ROS_ERROR( "The device on port %s was not recognized", port.c_str( ) );
	else if( result == RQ_UNRECOGNIZED_VERSION )
		ROS_ERROR( "Invalid device version" );
	else
		ROS_ERROR( "An unknown error occured while attempting to connect to device on port %s", port.c_str( ) );

	return false;
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

		if( getMotorRPM( ch1_vel, ch2_vel ) && getEncoderCount( ch1_pos, ch2_pos ) )
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
			joints.name.push_back( ch2_joint_name );

			joints.position.push_back( 2.0 * M_PI * ( double )ch1_pos / ( 4.0 * ( double )encoder_ppr ) );
			joints.position.push_back( 2.0 * M_PI * ( double )ch2_pos / ( 4.0 * ( double )encoder_ppr ) );

			joints.velocity.push_back( 2.0 * M_PI * ( double )ch1_vel / 60.0 );
			joints.velocity.push_back( 2.0 * M_PI * ( double )ch2_vel / 60.0 );

			joint_state_pub.publish( joints );

			last_joint_states_publish_time = current_time;
		}
	}
}

bool NxtGenDriver::getEncoderCount( int &enc1, int &enc2 )
{
	int result;

	result = dev.GetValue( _ABCNTR, 1, enc1 );
	if( !checkResult( result ) )
		return false;

	result = dev.GetValue( _ABCNTR, 2, enc2 );
	if( !checkResult( result ) )
		return false;

	return true;
}

bool NxtGenDriver::getMotorRPM( int &ch1, int &ch2 )
{
	int result;

	result = dev.GetValue( _ABSPEED, 1, ch1 );
	if( !checkResult( result ) )
		return false;

	result = dev.GetValue( _ABSPEED, 2, ch2 );
	if( !checkResult( result ) )
		return false;

	return true;
}

void NxtGenDriver::jointTrajCallback( const trajectory_msgs::JointTrajectoryConstPtr &msg )
{
	ROS_DEBUG( "Joint trajectory message received" );

	float rpm;
	int result;

	for( unsigned int i = 0; i < msg->joint_names.size( ); i++ )
	{
		if( msg->joint_names[i] == ch1_joint_name )
		{
			if( operating_mode == OPEN_LOOP_SPEED )
				rpm = 1000 * msg->points[0].velocities[i];
			else
			{
				rpm = 60.0 * msg->points[0].velocities[i] / ( 2.0 * M_PI );

				// Calculate rpm percentate for closed loop mode
				rpm = 1000.0 * rpm / ( float )ch1_max_motor_rpm;
			}

			if( invert ) rpm *= -1;

			ROS_INFO( "rpm: %f", rpm );

			// Round rpm value and send to channel 1
			result = dev.SetCommand( _GO, 1, ( int )( rpm + 0.5 ) );

			if( !checkResult( result) )
			{
				ROS_ERROR( "Sending command to %s failed", ch1_joint_name.c_str( ) );
			}
		}
		else if( msg->joint_names[i] == ch2_joint_name )
		{
			if( operating_mode == OPEN_LOOP_SPEED )
				rpm = 1000 * msg->points[0].velocities[i];
			else
			{
				rpm = 60.0 * msg->points[0].velocities[i] / ( 2.0 * M_PI );

				// Calculate rpm percentage for closed loop mode
	                        rpm = 1000.0 * rpm / ( float )ch2_max_motor_rpm;
			}

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

	int result;
	bool errors = false;

	if( running )
	{
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

		result = dev.GetValue( _BATAMPS, 2, battery2_amps );
		if( !checkResult( result ) )
			errors = true;

		result = dev.GetValue( _MOTAMPS, 1, motor1_amps );
		if( !checkResult( result ) )
			errors = true;

		result = dev.GetValue( _MOTAMPS, 2, motor2_amps );
		if( !checkResult( result ) )
			errors = true;

		result = dev.GetValue( _TEMP, 1, internal_ic_temp );
		if( !checkResult( result ) )
			errors = true;

		result = dev.GetValue( _TEMP, 2, heatsink_temp );
		if( !checkResult( result ) )
			errors = true;

		result = dev.GetValue( _MOTPWR, 1, motor1_power_output );
		if( !checkResult( result ) )
			errors = true;

		result = dev.GetValue( _MOTPWR, 2, motor2_power_output );
		if( !checkResult( result ) )
			errors = true;

		result = dev.GetValue( _FLTFLAG, fault_flag );
		if( !checkResult( result ) )
			errors = true;

		if( !errors )
			status.summary( diagnostic_msgs::DiagnosticStatus::OK, hardware_id + " is running" );
		else
			status.summary( diagnostic_msgs::DiagnosticStatus::ERROR, "Failed while reading diagnostics" );

		status.add( "Main battery voltage", ( double )main_battery_voltage / 10.0 );
		status.add( "Internal voltage", ( double )driver_voltage / 10.0 );
		status.add( "5V DSub connector voltage", ( double )dsub_5v_connector_voltage * 100.0 );
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

	// Channel 2 acceleration
	result = dev.SetConfig( _MAC, 2, config.acceleration );
	checkResult( result );

	// Channel 1 deceleration
	result = dev.SetConfig( _MDEC, 1, config.deceleration );
	checkResult( result );

	// Channel 2 deceleration
	result = dev.SetConfig( _MDEC, 2, config.deceleration );
	checkResult( result );

	// Channel 1 command linearity
	result = dev.SetConfig( _CLIN, 1, config.command_linearity );
	checkResult( result );

	// Channel 2 command linearity
	result = dev.SetConfig( _CLIN, 2, config.command_linearity );
	checkResult( result );
}

bool NxtGenDriver::checkResult( int result )
{
	switch( result )
	{
		case RQ_SUCCESS: // 0
			return true;

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
			ROS_ERROR( "Invalid response to the issued command." );
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

		case RQ_GET_CONFIG_FAILED: // 14
			ROS_ERROR( "Failed to get device configuration" );
			break;

		case RQ_GET_VALUE_FAILED: // 15
			ROS_ERROR( "Failed to get operating item configuration" );
			break;

		case RQ_SET_COMMAND_FAILED: // 16
			ROS_ERROR( "Failed to set device command" );
			break;

		default:
			ROS_ERROR( "An unknown error code was returned" );
			break;
	}

	return false;
}

} // namespace
