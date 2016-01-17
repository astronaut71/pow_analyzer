#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>

ros::Publisher pub;
sensor_msgs::Imu imu;

void imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
  imu.orientation.x = msg->orientation.y;  
  imu.orientation.y = msg->orientation.z; 
  imu.orientation.z = -msg->orientation.x; 
  imu.orientation.w =  msg->orientation.w; 

  imu.angular_velocity.x = msg->angular_velocity.y;
  imu.angular_velocity.y = msg->angular_velocity.z;
  imu.angular_velocity.z = -msg->angular_velocity.x;
 
  //imu.angular_velocity = msg->angular_velocity;
  //imu.linear_acceleration = msg->linear_acceleration;

  imu.linear_acceleration.x = msg->linear_acceleration.y;
  imu.linear_acceleration.y = msg->linear_acceleration.z;
  imu.linear_acceleration.z = -msg->linear_acceleration.x;

  /*imu.orientation_covariance[0] = 0.09;
  imu.orientation_covariance[4] = 0.09;
  imu.orientation_covariance[8] = 0.09;
  imu.angular_velocity_covariance[0] = 0.09;
  imu.angular_velocity_covariance[4] = 0.09;
  imu.angular_velocity_covariance[8] = 0.09;
  imu.linear_acceleration_covariance[0] = 0.09;
  imu.linear_acceleration_covariance[4] = 0.09;
  imu.linear_acceleration_covariance[8] = 0.09;*/

  imu.header = msg->header;
  imu.orientation_covariance = msg->orientation_covariance;
  imu.angular_velocity_covariance = msg->angular_velocity_covariance;
  imu.linear_acceleration_covariance = msg->linear_acceleration_covariance;
  imu.header.frame_id= "base_footprint"; // base_footprint for hector mapping base_link
  pub.publish(imu);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "converter");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("raw_imu_throttle", 1, imuCallback);   
  pub = nh.advertise<sensor_msgs::Imu>("imu2", 1);
 
 ros::spin();

}
