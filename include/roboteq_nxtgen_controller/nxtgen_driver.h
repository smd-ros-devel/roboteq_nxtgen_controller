#ifndef ROBOTEQ_NXTGEN_CONTROLLER_NXTGEN_DRIVER
#define ROBOTEQ_NXTGEN_CONTROLLER_NXTGEN_DRIVER

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_srvs/Empty.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <dynamic_reconfigure/server.h>
#include <roboteq_nxtgen_controller/RoboteqNxtGenConfig.h>

#include <string>
#include <vector>

#include "api_linux/Linux/RoboteqDevice.h"
#include "api_linux/Linux/ErrorCodes.h"
#include "api_linux/Linux/Constants.h"

namespace nxtgen_driver
{

namespace OperatingModes
{
	enum OperatingMode
	{
		ClosedLoopSpeed = 0, // User manual reports that closed loop speed mode is 2, but through testing I've determined it to be 0.
		OpenLoopSpeed = 1,
		//ClosedLoopSpeed = 2,
		ClosedLoopPosition = 3
	};
}
typedef OperatingModes::OperatingMode OperatingMode;

namespace DigitalActionLevels
{
	// Indicates whether a digital input or output is active high or active low
	enum DigitalActionLevel
	{
		ActiveHigh,
		ActiveLow
	};
}
typedef DigitalActionLevels::DigitalActionLevel DigitalActionLevel;

namespace DigitalInputActions
{
	// Possible actions for digital inputs
	enum DigitalInputAction
	{
		NoAction,
		SafetyStop,
		EmergencyStop,
		MotorStop,
		ForwardLimitSwitch,
		ReverseLimitSwitch,
		InvertDirection,
		RunMicroBasicScript,
		LoadHomeCounterValue
	};
}
typedef DigitalInputActions::DigitalInputAction DigitalInputAction;

namespace DigitalOutputActions
{
	// Possible actions for digital outputs
	enum DigitalOutputAction
	{
		NoAction,
		WhenMotorOn,
		MotorReversed,
		Overvoltage,
		Overtemperature,
		MirrorStatusLED,
		NoMOSFETFailure
	};
}
typedef DigitalOutputActions::DigitalOutputAction DigitalOutputAction;

struct DigitalInputConfig
{
	// Action for each digital input. Index 0 represents DIN1, index 1 represents DIN2, etc.
	// The size of the vector will indicate how many digital inputs the controller supports.
	std::vector<DigitalInputAction> inputs;

	// Level at which each digital input activates. The vector will be the same size as the
	// inputs array, and the indexes are also equivalent to the inputs array.
	std::vector<DigitalActionLevel> levels;

	// Motor channels to which the actions apply to. The vectors will be the same size as
	// the inputs and levels vectors, with indexes representing the digital input.
	std::vector<bool> motor1;
	std::vector<bool> motor2;
};

struct DigitalOutputConfig
{
	// Action for each digital output. Index 0 represents DOUT1, index 1 represents DOUT2, etc.
	// The size of the vector will indicate how many digital outputs the controller supports.
	std::vector<DigitalOutputAction> outputs;

	// Level the output will switch to when the digital output triggers. The size of the vector
	// and the indexes are equivalent to the outputs array.
	std::vector<DigitalActionLevel> levels;

	// Motor channels to which the actions apply to. The vectors will be the same size as
	// the inputs and levels vectors, with indexes representing the digital input.
	std::vector<bool> motor1;
	std::vector<bool> motor2;
};

class NxtGenDriver
{
	public:
		NxtGenDriver( ros::NodeHandle &nh );
		~NxtGenDriver( );
		bool init( );
		bool spin( );
		bool start( );
		void stop( );
		void update( );

		bool getDigitalInputConfig( DigitalInputConfig &config );
		bool getDigitalOutputConfig( DigitalOutputConfig &config );
		bool getEncoderCountAbs( int &enc1, int &enc2 );
		bool getEncoderCountRel( int &enc1, int &enc2 );
		bool getMotorRPM( int &ch1, int &ch2 );
		void publishJointStates( );
		static std::string operatingModeToStr( OperatingMode op_mode );
		static std::string digitalActionLevelToStr( DigitalActionLevel level );
		static std::string digitalInputActionToStr( DigitalInputAction action );
		static std::string digitalOutputActionToStr( DigitalOutputAction action );
		bool resetEncoderCount( );
		bool setOperatingMode( OperatingMode mode );

	private:
		void jointTrajCallback( const trajectory_msgs::JointTrajectoryConstPtr &msg );
		void deviceStatus( diagnostic_updater::DiagnosticStatusWrapper &status );
		void dynRecogCallback( roboteq_nxtgen_controller::RoboteqNxtGenConfig &config, uint32_t level );
		bool resetEncoderCallback( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res );

		bool checkResult( int result );

		RoboteqDevice dev;

		ros::NodeHandle nh;
		ros::Subscriber joint_traj_sub;
		ros::Publisher joint_state_pub;
		ros::ServiceServer reset_enc_srv;

		diagnostic_updater::Updater updater;

		std::string hardware_id;
		std::string port;
		std::string ch1_joint_name;
		std::string ch2_joint_name;

		bool invert;
		bool enable_watchdog;
		double watchdog_timeout; // timeout in seconds
		bool use_encoders;
		int encoder_type; // 0 = hall sensor; not 0 = optical
		int encoder_ppr;
		bool reset_encoder_count;
		OperatingMode operating_mode;
		int ch1_max_motor_rpm;
		int ch2_max_motor_rpm;

		bool running;
		int error_count;

		// Joints state publishing info
		bool publish_joint_states;
		double joint_states_publish_rate;
		double expected_joint_states_publish_time;
		ros::Time last_joint_states_publish_time;

		ros::Time last_time;
		ros::Time current_time;

		dynamic_reconfigure::Server<roboteq_nxtgen_controller::RoboteqNxtGenConfig> dyn_recog_server;
		dynamic_reconfigure::Server<roboteq_nxtgen_controller::RoboteqNxtGenConfig>::CallbackType dyn_recog_type;
};

} // namespace

#endif // ROBOTEQ_NXTGEN_CONTROLLER_NXTGEN_DRIVER
