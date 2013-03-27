#ifndef ROBOTEQ_NXTGEN_CONTROLLER_NXTGEN_DRIVER
#define ROBOTEQ_NXTGEN_CONTROLLER_NXTGEN_DRIVER

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <dynamic_reconfigure/server.h>
#include <roboteq_nxtgen_controller/RoboteqNxtGenConfig.h>

#include <string>

#include "api_linux/Linux/RoboteqDevice.h"
#include "api_linux/Linux/ErrorCodes.h"
#include "api_linux/Linux/Constants.h"

namespace nxtgen_driver
{

namespace OperatingModes
{
	enum OperatingMode
	{
		OPEN_LOOP_SPEED = 1,
		CLOSED_LOOP_SPEED = 2,
		CLOSED_LOOP_POSITION = 3
	};
}
typedef OperatingModes::OperatingMode OperatingMode;

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

	private:
		//bool getEncoderCountAbs( int &enc1, int &enc2 );
		//bool getEncoderCountRel( int &enc1, int &enc2 );
		bool getEncoderCount( int &enc1, int &enc2 );
		bool getMotorRPM( int &ch1, int &ch2 );

		void jointTrajCallback( const trajectory_msgs::JointTrajectoryConstPtr &msg );
		void publishJointStates( );
		void deviceStatus( diagnostic_updater::DiagnosticStatusWrapper &status );
		void dynRecogCallback( roboteq_nxtgen_controller::RoboteqNxtGenConfig &config, uint32_t level );

		bool checkResult( int result );

		RoboteqDevice dev;

		ros::NodeHandle nh;
		ros::Subscriber joint_traj_sub;
		ros::Publisher joint_state_pub;

		diagnostic_updater::Updater updater;

		std::string hardware_id;
		std::string port;
		std::string ch1_joint_name;
		std::string ch2_joint_name;

		bool invert;
		bool use_encoders;
		int encoder_type; // 0 = hall sensor; not 0 = optical
		int encoder_ppr;
		int operating_mode;
		int ch1_max_motor_rpm;
		int ch2_max_motor_rpm;

		bool running;

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
