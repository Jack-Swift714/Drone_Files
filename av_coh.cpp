#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <gnc_functions.hpp>




void scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	
	sensor_msgs::LaserScan current_scan;
  	current_scan = *msg;
	float avoidance_vector_x = 0; 
	float avoidance_vector_y = 0;
	float avoidance_vector_z = 0;
	float cohesion_vector_x = 0; 
	float cohesion_vector_y = 0;
	float cohesion_vector_z = 0;
	float height = 3;
	float deg2rad = (M_PI/180);
	bool avoid = false;
	bool cohere = false;

	//angle of lower ring is 0.1 radians below the midline
	//0.05 radians below
	//0 radians
	//0.05 radians above
	//0.1 radians above
	

	//determining what angle to move at and creating the avoidance vectors
	for(int i=1; i<current_scan.ranges.size(); i++)
	{
		float d0 = 3;
		float d1 = 4;
		float k = 0.5;
		//begin avoidance logic
		if(i >= 0 && i < (current_scan.ranges.size() / 5) && current_scan.ranges[i] < d0 && current_scan.ranges[i] > .35)
		{
			avoid = true;
			float x = cos(current_scan.angle_increment*i);
			float y = sin(current_scan.angle_increment*i);
			float z = sin(0.1);

			avoidance_vector_x = avoidance_vector_x + x*(3 - current_scan.ranges[i]);
			avoidance_vector_y = avoidance_vector_y + y*(3 - current_scan.ranges[i]);
			avoidance_vector_z = avoidance_vector_z + z*(3 - current_scan.ranges[i]);


		}
		if(i >= (current_scan.ranges.size() / 5) && i < (2 * current_scan.ranges.size() / 5) && current_scan.ranges[i] < d0 && current_scan.ranges[i] > .35)
		{
			avoid = true;
			float x = cos(current_scan.angle_increment*i);
			float y = sin(current_scan.angle_increment*i);
			float z = sin(0.05);

			avoidance_vector_x = avoidance_vector_x + x*(3 - current_scan.ranges[i]);
			avoidance_vector_y = avoidance_vector_y + y*(3 - current_scan.ranges[i]);
			avoidance_vector_z = avoidance_vector_z + z*(3 - current_scan.ranges[i]);

		}
		if(i >= (2 * current_scan.ranges.size() / 5) && i < (3 * current_scan.ranges.size() / 5) && current_scan.ranges[i] < d0 && current_scan.ranges[i] > .35)
		{
			avoid = true;
			float x = cos(current_scan.angle_increment*i);
			float y = sin(current_scan.angle_increment*i);

			avoidance_vector_x = avoidance_vector_x + x*(3 - current_scan.ranges[i]);
			avoidance_vector_y = avoidance_vector_y + y*(3 - current_scan.ranges[i]);

		}
		if(i >= (3 * current_scan.ranges.size() / 5) && i < (4 * current_scan.ranges.size() / 5) && current_scan.ranges[i] < d0 && current_scan.ranges[i] > .35)
		{
			avoid = true;
			float x = cos(current_scan.angle_increment*i);
			float y = sin(current_scan.angle_increment*i);
			float z = sin(0.05);

			avoidance_vector_x = avoidance_vector_x + x*(3 - current_scan.ranges[i]);
			avoidance_vector_y = avoidance_vector_y + y*(3 - current_scan.ranges[i]);
			avoidance_vector_z = avoidance_vector_z - z*(3 - current_scan.ranges[i]);

		}
		if(i >= (4 * current_scan.ranges.size() / 5) && i < current_scan.ranges.size() && current_scan.ranges[i] < d0 && current_scan.ranges[i] > .35)
		{
			avoid = true;
			float x = cos(current_scan.angle_increment*i);
			float y = sin(current_scan.angle_increment*i);
			float z = sin(0.1);

			avoidance_vector_x = avoidance_vector_x + x*(3 - current_scan.ranges[i]);
			avoidance_vector_y = avoidance_vector_y + y*(3 - current_scan.ranges[i]);
			avoidance_vector_z = avoidance_vector_z - z*(3 - current_scan.ranges[i]);

		}
		//begin cohesion logic
		if(i >= 0 && i < (current_scan.ranges.size() / 5) && current_scan.ranges[i] > d1 && current_scan.ranges[i] < current_scan.range_max)
		{
			cohere = true;
			float x = cos(current_scan.angle_increment*i);
			float y = sin(current_scan.angle_increment*i);
			float z = sin(0.1);

			cohesion_vector_x = cohesion_vector_x + x*(current_scan.ranges[i] - d1);
			cohesion_vector_y = cohesion_vector_y + y*(current_scan.ranges[i] - d1);
			cohesion_vector_z = cohesion_vector_z - z*(current_scan.ranges[i] - d1);


		}
		if(i >= (current_scan.ranges.size() / 5) && i < (2 * current_scan.ranges.size() / 5) && current_scan.ranges[i] > d1 && current_scan.ranges[i] < current_scan.range_max)
		{
			cohere = true;
			float x = cos(current_scan.angle_increment*i);
			float y = sin(current_scan.angle_increment*i);
			float z = sin(0.05);

			cohesion_vector_x = cohesion_vector_x + x*(current_scan.ranges[i] - d1);
			cohesion_vector_y = cohesion_vector_y + y*(current_scan.ranges[i] - d1);
			cohesion_vector_z = cohesion_vector_z - z*(current_scan.ranges[i] - d1);

		}
		if(i >= (2 * current_scan.ranges.size() / 5) && i < (3 * current_scan.ranges.size() / 5) && current_scan.ranges[i] > d1 && current_scan.ranges[i] < current_scan.range_max)
		{
			cohere = true;
			float x = cos(current_scan.angle_increment*i);
			float y = sin(current_scan.angle_increment*i);

			cohesion_vector_x = cohesion_vector_x + x*(current_scan.ranges[i] - d1);
			cohesion_vector_y = cohesion_vector_y + y*(current_scan.ranges[i] - d1);


		}
		if(i >= (3 * current_scan.ranges.size() / 5) && i < (4 * current_scan.ranges.size() / 5) && current_scan.ranges[i] > d1 && current_scan.ranges[i] < current_scan.range_max)
		{
			cohere = true;
			float x = cos(current_scan.angle_increment*i);
			float y = sin(current_scan.angle_increment*i);
			float z = sin(0.05);

			cohesion_vector_x = cohesion_vector_x + x*(current_scan.ranges[i] - d1);
			cohesion_vector_y = cohesion_vector_y + y*(current_scan.ranges[i] - d1);
			cohesion_vector_z = cohesion_vector_z + z*(current_scan.ranges[i] - d1);

		}
		if(i >= (4 * current_scan.ranges.size() / 5) && i < current_scan.ranges.size() && current_scan.ranges[i] > d1 && current_scan.ranges[i] < current_scan.range_max)
		{
			cohere = true;
			float x = cos(current_scan.angle_increment*i);
			float y = sin(current_scan.angle_increment*i);
			float z = sin(0.1);

			cohesion_vector_x = cohesion_vector_x + x*(current_scan.ranges[i] - d1);
			cohesion_vector_y = cohesion_vector_y + y*(current_scan.ranges[i] - d1);
			cohesion_vector_z = cohesion_vector_z + z*(current_scan.ranges[i] - d1);

		}




	}
	float current_heading = get_current_heading();
	avoidance_vector_x = avoidance_vector_x*cos((current_heading)*deg2rad) - avoidance_vector_y*sin((current_heading)*deg2rad);
	avoidance_vector_y = avoidance_vector_x*sin((current_heading)*deg2rad) + avoidance_vector_y*cos((current_heading)*deg2rad);
	height = height + avoidance_vector_z;
	cohesion_vector_x = cohesion_vector_x*cos((current_heading)*deg2rad) - cohesion_vector_y*sin((current_heading)*deg2rad);
	cohesion_vector_y = cohesion_vector_x*sin((current_heading)*deg2rad) + cohesion_vector_y*cos((current_heading)*deg2rad);
	height = height + cohesion_vector_z;
	if(avoid)
	{	/*
		if( sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)) > 3)
		{
			avoidance_vector_x = 3 * (avoidance_vector_x/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));
			avoidance_vector_y = 3 * (avoidance_vector_y/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));
		} */
		geometry_msgs::Point current_pos;
		current_pos = get_current_location();
		set_destination(current_pos.x - avoidance_vector_x, current_pos.y - avoidance_vector_y, height, 0);
		sleep(3);
	}
	if(cohere)
	{	/*
		if( sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)) > 3)
		{
			avoidance_vector_x = 3 * (avoidance_vector_x/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));
			avoidance_vector_y = 3 * (avoidance_vector_y/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));
		} */
		geometry_msgs::Point current_pos;
		current_pos = get_current_location();
		set_destination(current_pos.x + cohesion_vector_x, current_pos.y + cohesion_vector_y, height, 0);	
	}
	

}

int main(int argc, char **argv)
{
	//initialize ros 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle n;
	ros::Subscriber collision_sub = n.subscribe<sensor_msgs::LaserScan>("/spur/laser/scan", 1, scan_cb);
	//initialize control publisher/subscribers
	init_publisher_subscriber(n);

  	// wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
	wait4start();

	//create local reference frame 
	initialize_local_frame();

	//request takeoff
	takeoff(3);



	set_destination(0,0,3,0);
	sleep(10);
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

