/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  2013, Benedicte LC 		LLC.  
 *  Follows a wall.
 *
 *********************************************************************/

#include <ros/ros.h>
#include <turtlebot_node/TurtlebotSensorState.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/rate.h>
#include <iostream>
#include <stdio.h> 
#include <time.h>    

#define INF 100000.0;                       
            
using namespace std;   

//Function prototypes.
void bumperCallback(const turtlebot_node::TurtlebotSensorState::ConstPtr& msg);
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
void estimatePolicy(vector<float> laserVector);
void getSideLaserReading();
void getFrontLaserReading();

enum Policy {LEFT, RIGHT}; 

//Global variables.
uint8_t bumper;
float frontLaser, minimum;
vector<float> laserVector;	
Policy policy = LEFT;
bool firstTime = true;	

int main (int argc, char** argv){		
	
	ros::init(argc, argv, "wall_follower");	
	ros::NodeHandle nodeH;	
	ros::Rate loop_rate(10); //10 Hz.
	geometry_msgs::Twist twist;
	
	//Laser scan subscriber.
	ros::Subscriber laserSub = nodeH.subscribe<sensor_msgs::LaserScan>("/base_scan", 1000, laserCallback);
	//Bumper subscriber.
	ros::Subscriber bumperSub = nodeH.subscribe<turtlebot_node::TurtlebotSensorState>("/turtlebot_node/sensor_state", 1000, bumperCallback);
	//Moving.
	ros::Publisher movingPub = nodeH.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

	double forwardSpeed = 0.15;
	double turnSpeed = 0.1;
	//Twist set to navigate forward.
	twist.linear.x = forwardSpeed; 
	twist.linear.y = 0;
	twist.linear.z = 0;
    twist.angular.x = 0; 
    twist.angular.y = 0; 
    twist.angular.z = 0;
    
	//Keep the robot between 0.9m and 1.2m away from the wall.
	float safeDistance = 0.75; 
	float minWallDistance = 0.9;
	float maxWallDistance = 1.2;
	float farDistance = 2.0;
	bool foundWall = false;
	time_t start, now;
	
	while(ros::ok){
		
		int laserSize = laserVector.size(); 
		
		//Wait for first laser scan before starting.
		if(laserSize != 0){ 
			
			//Determine if left or right follower depending on closest wall.
			if(firstTime){	
				estimatePolicy(laserVector);
				firstTime = false;
			}
			
			getSideLaserReading();	
			getFrontLaserReading();	
			
			/*This means that the robot has been close enough to a wall
			 * at least once. The foundWall variable is used to detect
			 * if the wall is lost at some point, meaning a convex corner
			 * has been encountered.*/
			if(minimum <= maxWallDistance && !foundWall){
				ROS_INFO("Found a wall. \n");	
				foundWall = true;
			}	
			
			//Hit a wall. This doesn't work on Stage. Just in the real world.
			if(bumper != 0){
				ROS_INFO("Boom ouch! \n");		
				time(&start);
				time(&now);
				twist.linear.x = -forwardSpeed; //Move back.	
				twist.angular.z = 0.0; 
				while (difftime(now, start) < 1.5){ //Go back for 1.5 seconds.
				
					time(&now);
					movingPub.publish(twist); 								
				}
			}
			
			//There is an obstacle ahead. Turn around until gone.
			else if(frontLaser < safeDistance){
				
				ROS_INFO("Wall in front!!! \n");		
				twist.linear.x = 0.0; 	
				
				if(policy == LEFT){					
					twist.angular.z = -turnSpeed * 3; //Turn away, right.				
				}
				else{					
					twist.angular.z = turnSpeed * 3; //Turn away, left.				
				} 
				
				while (frontLaser < safeDistance
					|| minimum < safeDistance){ //Turn until safe.
					
					movingPub.publish(twist); 		
					getFrontLaserReading();	
					getSideLaserReading();	
					loop_rate.sleep();
					ros::spinOnce();
				}			
			}
			
			//Wall is very far or lost to a convex corner.
			else if(minimum > farDistance){
				//Wall is very far. Need to get closer.
				if(!foundWall){ 
					ROS_INFO("Wall very far. \n");
					if(policy == LEFT){					
						twist.angular.z = 0.01; //Turn towards, left.				
					}
					else{					
						twist.angular.z = -0.01; //Turn towards, right.				
					} 	
				
					twist.linear.x = forwardSpeed; //Go forward.
					movingPub.publish(twist); 
				}
				//Lost the wall! Convex corner!
				else{ 
					ROS_INFO("Convex corner. \n");
					
					if(policy == LEFT){					
						twist.angular.z = 0.9*turnSpeed; //Turn towards, left.	0.9*turnSpeed is good for simulator & 1.2 is good for real.	
					}
					else{					
						twist.angular.z = -0.9*turnSpeed; //Turn towards, right.				
					} 
					
					twist.linear.x = 1.1*forwardSpeed; //Go forward.
					movingPub.publish(twist); 
					
				}				
			}
			
			//Wall on the left is too close. 
			else if(minimum < minWallDistance && policy == LEFT){	
			
				ROS_INFO("Wall too close. Left policy. \n");
				twist.angular.z = -turnSpeed; //Turn away, right.
				twist.linear.x = forwardSpeed;
				movingPub.publish(twist); 
			}
			
			//Wall on the left is too far.
			else if(minimum > maxWallDistance && policy == LEFT){			
				
				ROS_INFO("Wall too far. Left policy. \n");
				twist.angular.z = turnSpeed; //Turn towards, left.	
				twist.linear.x = forwardSpeed; 	
				movingPub.publish(twist); 
			}		
			
			//Wall on the left is too close. 
			else if(minimum < minWallDistance && policy == RIGHT){	
			
				ROS_INFO("Wall too close. Right policy.\n");
				twist.angular.z = turnSpeed; //Turn away, left.
				twist.linear.x = forwardSpeed;
				movingPub.publish(twist); 
			}
			
			//Wall on the left is too far.
			else if(minimum > maxWallDistance && policy == RIGHT){			
				
				ROS_INFO("Wall too far. Right policy.\n");
				twist.angular.z = -turnSpeed; //Turn towards, right.	
				twist.linear.x = forwardSpeed; 	
				movingPub.publish(twist); 			
			}	
					
			//Within good band. Move forward.
			else{
				ROS_INFO("Going forward. \n");
				twist.angular.z = 0.0;			
				twist.linear.x = forwardSpeed; 				
				movingPub.publish(twist); 
			}
		}
		
		loop_rate.sleep();
		ros::spinOnce();
	}
	
	return 0;
}



//Fetch the sensor state containing the bumper state.
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	
	laserVector = msg->ranges;	
}

//Fetch the sensor state containing the bumper state.
void bumperCallback(const turtlebot_node::TurtlebotSensorState::ConstPtr& msg){
	
	bumper = msg->bumps_wheeldrops;	
}

void estimatePolicy(vector<float> laserVector){ //Estimate if the wall is to the left or right.	
	
	float left = 0.0;
	float right = 0.0;
	int middle = laserVector.size()/2; 
		
	//Add all the laserscan points for the left.
	for(int i = 0; i < middle ; i++){
		right += laserVector[i];
	}
	for(int i = middle; i < laserVector.size(); i++){
		left += laserVector[i];
	}
		
	//Get their average value for the left and right sides.
	if( (right / middle) < (left / (laserVector.size() - middle)) ){
		ROS_INFO("Wall is to the right");
		policy = RIGHT;
	}
	else{
		ROS_INFO("Wall is to the left");
		policy = LEFT;
	}	
}
	
//Get the minimum right or left laser reading.
void getSideLaserReading(){
	minimum = INF; //Initial value.
	int laserSize = laserVector.size(); 		
	
	//Find the minimum laser value on the left or right; represents how close the wall is.
	if (policy == RIGHT){
		for(int i = 0 ; i < (int)laserSize/3 ; i++){
			if (minimum > laserVector[i]){
				minimum = laserVector[i];
			}
		}
	}
	else { 
		for(int i = (int)(2*laserSize/3) ; i < laserSize ; i++){
			if (minimum > laserVector[i]){
				minimum = laserVector[i];
			}
		}
	}
}

//Get the minimum front laser reading.
void getFrontLaserReading(){	
	frontLaser = INF; //Initial value.
	int laserSize = laserVector.size(); 
		
	//Find minimum laser value in front; represents how close an obstacle is.
	for(int i = (int)(laserSize/3) ; i < (int)(2*laserSize/3) ; i++){
			if (frontLaser > laserVector[i]){
				frontLaser = laserVector[i];
		}
	}	
}

