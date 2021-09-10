#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <gnc_functions.hpp>

void scan_cb_front(const sensor_msgs::Range::ConstPtr& msg)
{
	
	sensor_msgs::Range current_Distance;

  	current_Distance = *msg;
	float avoidance_vector = 0; 
	float cohesion_vector = 0;
	bool avoid = false;
	bool cohere = false;

	//std::cout << current_Distance.range;
	//std::cout << current_Distance.max_range;

	if (current_Distance.range > 5 && current_Distance.range < current_Distance.max_range){

		cohesion_vector = 2;
		cohere = true;

	}
	float current_heading = get_current_heading();
	float deg2rad = (M_PI/180); 
	if(cohere) {
		geometry_msgs::Point current_pos;
		current_pos = get_current_location();
		set_destination(current_pos.x, cohesion_vector + current_pos.y, 2, 0);

	}
	if (current_Distance.range < 2 && current_Distance.range > current_Distance.min_range){

		avoidance_vector = 2;
		avoid = true;

	}
	if(avoid) {
		geometry_msgs::Point current_pos;
		current_pos = get_current_location();
		set_destination(current_pos.x, current_pos.y - avoidance_vector, 2, 0);

	}
}
void scan_cb_back(const sensor_msgs::Range::ConstPtr& msg)
{
	
	sensor_msgs::Range current_Distance;

  	current_Distance = *msg;
	float avoidance_vector = 0; 
	float cohesion_vector = 0;
	bool avoid = false;
	bool cohere = false;

	//std::cout << current_Distance.range;
	//std::cout << current_Distance.max_range;

	if (current_Distance.range > 5 && current_Distance.range < current_Distance.max_range){

		cohesion_vector = 2;
		cohere = true;

	}
	float current_heading = get_current_heading();
	float deg2rad = (M_PI/180); 
	if(cohere) {
		geometry_msgs::Point current_pos;
		current_pos = get_current_location();
		set_destination(current_pos.x, current_pos.y - cohesion_vector, 2, 0);

	}
	if (current_Distance.range < 2 && current_Distance.range > current_Distance.min_range){

		avoidance_vector = 2;
		avoid = true;

	}
	if(avoid) {
		geometry_msgs::Point current_pos;
		current_pos = get_current_location();
		set_destination(current_pos.x, current_pos.y + avoidance_vector, 2, 0);

	}
}
void scan_cb_left(const sensor_msgs::Range::ConstPtr& msg)
{
	
	sensor_msgs::Range current_Distance;

  	current_Distance = *msg;
	float avoidance_vector = 0; 
	float cohesion_vector = 0;
	bool avoid = false;
	bool cohere = false;

	//std::cout << current_Distance.range;
	//std::cout << current_Distance.max_range;

	if (current_Distance.range > 5 && current_Distance.range < current_Distance.max_range){

		cohesion_vector = 2;
		cohere = true;

	}
	float current_heading = get_current_heading();
	float deg2rad = (M_PI/180); 
	if(cohere) {
		geometry_msgs::Point current_pos;
		current_pos = get_current_location();
		set_destination(current_pos.x - cohesion_vector, current_pos.y, 2, 0);

	}
	if (current_Distance.range < 2 && current_Distance.range > current_Distance.min_range){

		avoidance_vector = 2;
		avoid = true;

	}
	if(avoid) {
		geometry_msgs::Point current_pos;
		current_pos = get_current_location();
		set_destination(current_pos.x + avoidance_vector, current_pos.y, 2, 0);

	}
}
void scan_cb_right(const sensor_msgs::Range::ConstPtr& msg)
{
	
	sensor_msgs::Range current_Distance;

  	current_Distance = *msg;
	float avoidance_vector = 0; 
	float cohesion_vector = 0;
	bool avoid = false;
	bool cohere = false;

	//std::cout << current_Distance.range;
	//std::cout << current_Distance.max_range;

	if (current_Distance.range > 5 && current_Distance.range < current_Distance.max_range){

		cohesion_vector = 2;
		cohere = true;

	}
	float current_heading = get_current_heading();
	float deg2rad = (M_PI/180); 
	if(cohere) {
		geometry_msgs::Point current_pos;
		current_pos = get_current_location();
		set_destination(current_pos.x + cohesion_vector, current_pos.y, 2, 0);

	}
	if (current_Distance.range < 2 && current_Distance.range > current_Distance.min_range){

		avoidance_vector = 2;
		avoid = true;

	}
	if(avoid) {
		geometry_msgs::Point current_pos;
		current_pos = get_current_location();
		set_destination(current_pos.x - avoidance_vector, current_pos.y, 2, 0);

	}
}

int main(int argc, char **argv)
{
	//initialize ros 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle n;
	ros::Subscriber collision_sub_front = n.subscribe<sensor_msgs::Range>("/drone1/sensor/sonar/front", 1, scan_cb_front);
	ros::Subscriber collision_sub_back = n.subscribe<sensor_msgs::Range>("/drone1/sensor/sonar/back", 1, scan_cb_back);
	ros::Subscriber collision_sub_left = n.subscribe<sensor_msgs::Range>("/drone1/sensor/sonar/left", 1, scan_cb_left);
	ros::Subscriber collision_sub_right = n.subscribe<sensor_msgs::Range>("/drone1/sensor/sonar/right", 1, scan_cb_right);
	//initialize control publisher/subscribers
	init_publisher_subscriber(n);

  	// wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
	wait4start();

	//create local reference frame 
	initialize_local_frame();

	//request takeoff
	takeoff(2);


	set_destination(0,0,2,0);
	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0);
	int counter = 0;
	while(ros::ok())
	{
		
		ros::spinOnce();
		rate.sleep();
		
	
	
	}

	return 0;
}

