#include <ros/ros.h>
#include "roboteq_nxtgen_controller/nxtgen_driver.h"

int main( int argc, char **argv )
{
	ros::init( argc, argv, "nxtgen_driver_node" );

	ros::NodeHandle nh;

	nxtgen_driver::NxtGenDriver nxt( nh );

	nxt.spin( );

	return 0;
}
