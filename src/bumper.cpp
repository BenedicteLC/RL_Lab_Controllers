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
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
void estimateWallDist(ros::Duration elapsedTime, double forwardSpeed);
void calcDistance();

uint8_t bumper;
const double speedThreshold = 0.0;
double currentSpeed, timeAtSpeed, setSpeed = 0.0;
double traveledDistance = 0.0;
ros::Time startTime;

int main (int argc, char** argv){		
	
	ros::Time start, end;
	ros::init(argc, argv, "bumper");	
	ros::NodeHandle nodeH;	
	ros::Rate loop_rate(10); //10 Hz.
	geometry_msgs::Twist twist;
	
	//Bumper subscriber.
	ros::Subscriber bumperSub = nodeH.subscribe<turtlebot_node::TurtlebotSensorState>("/turtlebot_node/sensor_state", 1000, bumperCallback);
	//Odometry subscriber.
	ros::Subscriber odomSub = nodeH.subscribe<nav_msgs::Odometry>("/odom", 1000, odomCallback);
	//Moving.
	ros::Publisher movingPub = nodeH.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

	//Twist set to navigate forward.
	bool done = false;
	double forwardSpeed = 0.15;
	twist.linear.x = forwardSpeed; 
	twist.linear.y = 0;
	twist.linear.z = 0;
    twist.angular.x = 0; 
    twist.angular.y = 0; 
    twist.angular.z = 0;
    
    startTime = ros::Time::now(); //Time when the robot starts moving.
	start = ros::Time::now(); //Time when the robot starts moving. Less accurate calculation.
	
	while(ros::ok){
		if(bumper != 0 && !done){ //Hit a wall.
			ROS_INFO("BOOM OUCH");		
				
			end = ros::Time::now(); 			
			done = true;
			calcDistance(); //Estimate distance traveled. More accurate calculation.
			estimateWallDist(end - start, forwardSpeed); //Less accurate calculation.					
		}
		else if (!done){ //Go forward until the robot bumps into a wall.
			ROS_INFO("Going forward");		
			
			calcDistance(); //Estimate distance traveled. More accurate calculation.					
			movingPub.publish(twist); //Move forward.
		}
		else{ //Done. 
			ROS_INFO("Algo B. The more accurate distance is estimated to %fm.", traveledDistance);
			break;			
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

//Fetch the sensor state containing the bumper state.
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	
	double xVelocity, yVelocity;	
	
	//Get current robot speed.
	xVelocity = msg->twist.twist.linear.x;
    yVelocity = msg->twist.twist.linear.y;
    currentSpeed = sqrt(pow(xVelocity, 2.0) + pow(yVelocity, 2.0));   
}

//Estimate the distance to the wall based on the time taken to hit it.
//Algorithm A.
void estimateWallDist(ros::Duration elapsedTime, double forwardSpeed){
	
	double time = elapsedTime.toSec();
	double distance = forwardSpeed * time; //Need better measure to consider acceleration.
	
	ROS_INFO("Algo A. The less accurate distance is estimated to %fm.", distance);	
}

//Calculate distance traveled at a certain speed.
//Algorithm B.
void calcDistance(){
	ros::Time currentTime = ros::Time::now(); 
    
	if(setSpeed == 0){ //Initialize.
		setSpeed = currentSpeed;
		startTime = currentTime;
	}
	else if (fabs(setSpeed - currentSpeed) > speedThreshold && currentSpeed != 0){ //Big enough speed difference.
		timeAtSpeed = (currentTime - startTime).toSec(); //Time spent at this speed. 
		traveledDistance += setSpeed * timeAtSpeed; //Distance traveled at set speed.
		
		//Start a new timer for this new speed.
		startTime = currentTime;
		setSpeed = currentSpeed;
	}
}

