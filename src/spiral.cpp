/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  2013, Benedicte LC 		LLC.  
 *  Estimate the distance between the current robot position
 *  and a wall lying straight ahead.
 *
 *********************************************************************/

#include <ros/ros.h>
#include <turtlebot_node/TurtlebotSensorState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/rate.h>
#include <iostream>
#include <stdio.h> 
//#include <string.h>                           
            
using namespace std;    

void bumperCallback(const turtlebot_node::TurtlebotSensorState::ConstPtr& msg);

uint8_t bumper;

int main (int argc, char** argv){			
	
	ros::init(argc, argv, "bumper");	
	ros::NodeHandle nodeH;	
	ros::Rate loop_rate(10); //10 Hz.
	geometry_msgs::Twist twist;
	
	//Bumper subscriber.
	ros::Subscriber bumperSub = nodeH.subscribe<turtlebot_node::TurtlebotSensorState>("/turtlebot_node/sensor_state", 1000, bumperCallback);
	
	//Moving.
	ros::Publisher movingPub = nodeH.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

	//Twist set to navigate forward.	
	int cycle = 250;
	double delta; 
	double forwardSpeed = 0.5;
	twist.linear.x = forwardSpeed; 
	twist.linear.y = 0;
	twist.linear.z = 0;
    twist.angular.x = 0; 
    twist.angular.y = 0; 
    twist.angular.z = 0;
    
	
	while(ros::ok){
		if(bumper != 0){ //Hit a wall.
			ROS_INFO("Done.");						
			break;
					
		}
		else { //Move in a spiral until the robot bumps into a wall.
			ROS_INFO("Moving in a spiral.");	
			cycle+=2;
			delta = 500.0 / (double)cycle;
			twist.angular.z = delta;
				
			movingPub.publish(twist); //Move in a spiral.
			
		}
	
		loop_rate.sleep();
		ros::spinOnce();
	}
	
	return 0;
}

//Fetch the sensor state containing the bumper state.
void bumperCallback(const turtlebot_node::TurtlebotSensorState::ConstPtr& msg){
	
	bumper = msg->bumps_wheeldrops;	
}

