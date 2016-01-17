#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include <iostream>
#include <fstream>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "/opt/ros/fuerte/stacks/bullet/include/LinearMath/btScalar.h"
#include "/opt/ros/fuerte/stacks/bullet/include/LinearMath/btQuaternion.h"
#include "/opt/ros/fuerte/stacks/bullet/include/LinearMath/btMatrix3x3.h"
#include "std_msgs/Float64.h"


double time_prev;
double prev_theta = 0;
double prev_x = 0;
double prev_y = 0;
int count = 0;
int count2 = 0;
int speed_count = 0;
double speed_sum = 0;
double speed_sum_theta = 0;
int zero_count = 0;
std::ofstream myfile;


void speedCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{

    btScalar r,p,y;
    btQuaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
    btMatrix3x3(q).getEulerYPR(y,p,r);
    std_msgs::Float64 yaw;
    yaw.data = y;
    //ROS_INFO("Yaw: %f", yaw.data*57.29);


    if (count2 == 0){
        time_prev = ros::Time::now().toSec();
        count2=1;
    }

    double time_now = ros::Time::now().toSec();

    //double curr_theta = msg->theta;
    //double curr_x = msg->x;
    //double curr_y = msg->y;

    double curr_theta = yaw.data;
    double curr_x = msg->pose.pose.position.x;
    double curr_y = msg->pose.pose.position.y;

    //ROS_INFO(" before curr_x = %f, curr_y = %f", curr_x, curr_y);

    if (curr_x < 0){
        curr_x = curr_x * -1;
    }
    if (curr_y < 0){
        curr_y = curr_y * -1;
    }
    //ROS_INFO("after curr_x = %f, curr_y = %f", curr_x, curr_y);
    double time_dif = time_now - time_prev;
    if (prev_x != 0 || prev_y != 0){
        if (time_dif != 0){
            double dist_theta = (curr_theta - prev_theta);
        if (dist_theta < 0){
               dist_theta = dist_theta * -1;
            }
            double dist_x = (curr_x - prev_x);
            if (dist_x < 0){
                dist_x = dist_x * -1;
            }
            double dist_y = (curr_y - prev_y);
            if (dist_y < 0){
                dist_y = dist_y * -1;
            }
            double dist_tot = sqrt((dist_x * dist_x) + (dist_y * dist_y));// * 10;
            double speed = (dist_tot / time_dif);// * 3.6;
            double speed_theta = dist_theta /time_dif;
		
            
           	
		speed_count++;
		
            	speed_sum = speed_sum + speed;
            	speed_sum_theta = speed_sum_theta + speed_theta;
            	
           if (speed_count == 2){
                speed = speed_sum / speed_count;
                speed_theta = speed_sum_theta / speed_count;
                if (zero_count > 7){
                    speed = 0;
                }
                speed_count = 0;
                speed_sum = 0;
                speed_sum_theta = 0;
                zero_count = 0;
            
                count = count + 1;
		ROS_INFO("linear speed= %f, anglular speed=  %f", speed, speed_theta*57.29); 		
		myfile << speed <<  " 		" << speed_theta*57.29 << "\n";   

            }

        }
    }
		
    prev_theta = curr_theta;
    prev_x = curr_x;
    prev_y = curr_y;

    time_prev = time_now;
    
}

int main(int argc, char **argv)
{
    myfile.open ("/home/bojan/Desktop/Seegway/Speed/AMCL/seegway-amcl.txt");
    myfile << "speed" << " 		" << "speed_theta" << "\n"; 
    ros::init(argc, argv, "speed_node");
    ros::NodeHandle n;


    ros::Subscriber sub = n.subscribe("/amcl_pose", 100, speedCallback);

    ros::spin(); /*while(ros::ok())
    {
	ros::spinOnce();
	ros::Duration(0.9).sleep();
    }*/
    //myfile.close();
    return 0;
}

