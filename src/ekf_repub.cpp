#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

ros::Publisher pub;
nav_msgs::Odometry odom;

double time_last, xlast = 0, ylast = 0, yaw_last = 0, delta_t, curr_yaw;

void initialize(void)
{
  odom.header.frame_id = "map";
  odom.child_frame_id = "base_link ";
  odom.pose.pose.position.z = 0;
}

double normalize(double in)
{
if(in < -M_PI) return in + 2*M_PI;
else if(in > M_PI) return in - 2*M_PI;
else return in;
}

void poseCallback(const nav_msgs::OdometryConstPtr& msg)
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

void setup()
{
  odom.twist.twist.linear.z = odom.twist.twist.angular.x = odom.twist.twist.angular.y = 0;
time_last = ros::Time::now().toSec();
  
odom.pose.covariance[0]  = 0.009;
odom.pose.covariance[7]  = 0.009;
odom.pose.covariance[14] = 9999;
odom.pose.covariance[21] = 9999;
odom.pose.covariance[28] = 9999;
odom.pose.covariance[35] = 0.009;



/*odom.pose.covariance[0]  = 0.0000001;
odom.pose.covariance[7]  = 0.0000001;
odom.pose.covariance[14] = 99999999999;
odom.pose.covariance[21] = 99999999999;
odom.pose.covariance[28] = 99999999999;
odom.pose.covariance[35] = 0.0000001;*/


/*odom.pose.covariance[0]  = 0.9;
odom.pose.covariance[7]  = 0.9;
odom.pose.covariance[14] = 999;
odom.pose.covariance[21] = 999;
odom.pose.covariance[28] = 999;
odom.pose.covariance[35] = 0.9;*/




odom.twist.covariance=odom.pose.covariance;// = msg->pose.covariance;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hector_odometry");
  initialize();
  ros::NodeHandle nh;

setup();
  ros::Subscriber sub = nh.subscribe("odom", 1, poseCallback);
  pub = nh.advertise<nav_msgs::Odometry>("wheelodom", 1);

  ros::spin();
}
