#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>

ros::Publisher pub, pub2; sensor_msgs::LaserScan scanout;
nav_msgs::Odometry odom, odom_raw, amcl_odometry;
geometry_msgs::TransformStamped odom_trans;

void initialize(void)
{
  odom.header.frame_id = "map"; odom_raw.header.frame_id = "raw_frame"; 
  odom.child_frame_id = "amcl_base_link"; odom_raw.child_frame_id = "raw_odometry";
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


void odom1Callback(const nav_msgs::OdometryConstPtr& msg)
{
 	//first, we'll publish the transform over tf
    	odom_trans.transform.translation.x = odom_raw.pose.pose.position.x  =  msg->pose.pose.position.x;
    	odom_trans.transform.translation.y = odom_raw.pose.pose.position.y  =  msg->pose.pose.position.y;

    	odom_trans.transform.rotation = odom_raw.pose.pose.orientation = msg->pose.pose.orientation;
        odom_trans.header.stamp = odom_raw.header.stamp = ros::Time::now();
        pub2.publish(odom_raw);
	
	static tf::TransformBroadcaster br;
 	
}


void odomCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
scanout.header.frame_id = "laser";
scanout.header.stamp = ros::Time::now();
scanout.header.seq = msg->header.seq;
scanout.angle_min = msg->angle_min;
scanout.angle_max = msg->angle_max;
scanout.angle_increment = msg->angle_increment;
scanout.time_increment = msg->time_increment;
scanout.scan_time = msg->scan_time;
scanout.range_min = msg->range_min;
scanout.range_max = msg->range_max;

for(int i=0; i<msg->ranges.size(); i++) scanout.ranges[i] = msg->ranges[i];
pub.publish(scanout);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ekf_encoders_odom");
  ros::NodeHandle nh;
  initialize();

scanout.ranges.resize(800);

  ros::Subscriber sub1 = nh.subscribe("odom_throttle", 1, odom1Callback);
  ros::Subscriber sub = nh.subscribe("scan_throttle", 1, odomCallback);
  pub = nh.advertise<sensor_msgs::LaserScan>("scan_2", 1);
  pub2 = nh.advertise<nav_msgs::Odometry>("raw_odometry", 1);

 ros::spin();
return 0;
}
