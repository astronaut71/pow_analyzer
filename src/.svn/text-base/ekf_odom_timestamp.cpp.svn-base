#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <fstream>

std::ofstream myfile;
using namespace std;

ros::Publisher pub;
nav_msgs::Odometry odom;

double time_last, xlast = 0, ylast = 0, yaw_last = 0, delta_t, curr_yaw, vel_linear, vel_angular;
double first_time; double timestamp; bool datum_time_rec = 0;

double normalize(double in)
{
if(in < -M_PI) return in + 2*M_PI;
else if(in > M_PI) return in - 2*M_PI;
else return in;
}

void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{

  if(!datum_time_rec) { 
	first_time = time_last = ros::Time::now().toSec();
	datum_time_rec = 1;
 			}
  odom.pose.pose.position.x = msg->pose.pose.position.y;
  odom.pose.pose.position.y = -msg->pose.pose.position.x;

  curr_yaw = normalize(tf::getYaw(msg->pose.pose.orientation) - 0.5*M_PI);
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(curr_yaw);
  odom.header.stamp = msg->header.stamp;

//Speed calculation
delta_t = ros::Time::now().toSec() - time_last;
odom.twist.twist.linear.x = (msg->pose.pose.position.x-xlast)/delta_t;
odom.twist.twist.linear.y = (msg->pose.pose.position.y-ylast)/delta_t;
odom.twist.twist.angular.z = normalize(curr_yaw-yaw_last)/delta_t;

//linear and angular speed
vel_linear = sqrt(odom.twist.twist.linear.x*odom.twist.twist.linear.x + odom.twist.twist.linear.y*odom.twist.twist.linear.y);
vel_angular = fabs(odom.twist.twist.angular.z*57.29);
timestamp = (ros::Time::now().toSec()-first_time);  ROS_INFO("AT TIME %f", timestamp);
ROS_INFO("linear speed=  %f, anglular speed=  %f", vel_linear, vel_angular); 
myfile << timestamp << "	" << vel_linear <<  " 		" << vel_angular<< "\n";  


  pub.publish(odom);
  xlast = msg->pose.pose.position.x; ylast = msg->pose.pose.position.y; yaw_last = curr_yaw;
  time_last = ros::Time::now().toSec();
}

void setup()
{
  odom.twist.twist.linear.z = odom.twist.twist.angular.x = odom.twist.twist.angular.y = 0;
  

 odom.header.frame_id = "ekf_frame";
 //odom.child_frame_id = "base_link";
 odom.child_frame_id = "ekf_footprint";
  odom.pose.pose.position.z = 0;

}

int main(int argc, char **argv)
{
  
  myfile.open ("/home/bojan/svn/POW/ROS/Fuerte/Data/EKF-Hector-POW/Run1/U1-imudef1.tx");
  myfile << "timestamp" << "	" << "speed" << " 		" << "speed_theta" << "\n"; 
  ros::init(argc, argv, "ekf_odometry");

  ros::NodeHandle nh;

setup();
  ros::Subscriber sub = nh.subscribe("/robot_pose_ekf/odom", 1, poseCallback);
  pub = nh.advertise<nav_msgs::Odometry>("ekf_odom", 1);

  ros::spin();
}
