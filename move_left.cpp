#include <gnc_functions.hpp>
#include <math.h>

int main(int argc, char** argv)
{
	//initialize ros 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle gnc_node("~");
	
	//initialize control publisher/subscribers
	init_publisher_subscriber(gnc_node);

  	// wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
	set_mode("GUIDED");

	//create local reference frame 
	initialize_local_frame();

	//request takeoff
	takeoff(3);

	set_destination(-25,0,3,0);

	return 0;
}