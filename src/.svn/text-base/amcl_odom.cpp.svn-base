#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <fstream>

ros::Publisher pub, pub2;
nav_msgs::Odometry odom, odom_raw, amcl_odometry;
geometry_msgs::TransformStamped odom_trans;
double x, y, yabs;

double first_time; double timestamp;

std::ofstream myfile;
using namespace std;

void initialize(void)
{
  odom.header.frame_id = "map"; odom_raw.header.frame_id = "raw_frame"; //amcl_odometry.header.frame_id = "amcl_frame";
  odom.child_frame_id = "amcl_base_link"; odom_raw.child_frame_id = "raw_odometry"; //amcl_odometry.child_frame_id = "amcl_odometry";
  odom.pose.pose.position.z = odom_raw.pose.pose.position.z = odom_trans.transform.translation.z = 0;

    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
}

double normalize(double in)
{
	if(in < -M_PI) return in + 2*M_PI;
	else if(in > M_PI) return in - 2*M_PI;
	else return in;
}


double time_last, xlast = 0, ylast = 0, yaw_last = 0, delta_t, curr_yaw;

void odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  //first, we'll publish the transform over tf
    odom_trans.transform.translation.x = odom_raw.pose.pose.position.x  =  msg->pose.pose.position.x;
    odom_trans.transform.translation.y = odom_raw.pose.pose.position.y  =  msg->pose.pose.position.y;

    //odom_trans.transform.translation.x = amcl_odometry.pose.pose.position.x = msg->pose.pose.position.x;
    //odom_trans.transform.translation.y = amcl_odometry.pose.pose.position.y =  msg->pose.pose.position.y;

    odom_trans.transform.rotation = odom_raw.pose.pose.orientation = msg->pose.pose.orientation;

    //odom_trans.transform.rotation = amcl_odometry.pose.pose.orientation = msg->pose.pose.orientation;
	odom_raw.twist = msg->twist;
	

	
        odom_trans.header.stamp = odom_raw.header.stamp = ros::Time::now();
        pub2.publish(odom_raw);
	
    //pub.publish(amcl_odometry)
	static tf::TransformBroadcaster br;
    //br.sendTransform(odom_trans);
	
}

void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  odom.pose.pose.position.x = msg->pose.pose.position.x;
  odom.pose.pose.position.y = msg->pose.pose.position.y;
  odom.pose.pose.orientation = msg->pose.pose.orientation;
  odom.header.stamp = msg->header.stamp;


//Speed calculation
  delta_t = ros::Time::now().toSec() - time_last;
  odom.twist.twist.linear.x = (msg->pose.pose.position.x-xlast)/delta_t;
  odom.twist.twist.linear.y = (msg->pose.pose.position.y-ylast)/delta_t;
  curr_yaw = tf::getYaw(msg->pose.pose.orientation);
  odom.twist.twist.angular.z = normalize(curr_yaw-yaw_last)/delta_t;


  pub.publish(odom);
  xlast = msg->pose.pose.position.x; ylast = msg->pose.pose.position.y; yaw_last = curr_yaw;
  time_last = ros::Time::now().toSec();
}

int main(int argc, char **argv)
{

  //myfile.open ("/home/bojan/svn/GDS/PreliminaryResults/Data/odom_ekf_imu_malku2.txt");
  //myfile << "pos_x" << "	" << "pos_y" << "\n"; 

  ros::init(argc, argv, "ekf_encoders_odom");
  ros::NodeHandle nh;
  initialize();

  ros::Subscriber sub = nh.subscribe("odom", 1, odomCallback);

  ros::Subscriber sub2 = nh.subscribe("amcl_pose", 1, poseCallback);
  pub = nh.advertise<nav_msgs::Odometry>("amcl_odometry", 1);
  pub2 = nh.advertise<nav_msgs::Odometry>("raw_odometry", 1);
 
  ros::spin();
 
}
