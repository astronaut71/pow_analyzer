#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include <iostream>
#include <fstream>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "/opt/ros/fuerte/stacks/bullet/include/LinearMath/btScalar.h"
#include "/opt/ros/fuerte/stacks/bullet/include/LinearMath/btQuaternion.h"
#include "/opt/ros/fuerte/stacks/bullet/include/LinearMath/btMatrix3x3.h"
#include "std_msgs/Float64.h"

#include <tf/transform_listener.h>
#include "geometry_msgs/Quaternion.h"


std::ofstream myfile;
using namespace std;

double hypotenuse(double xa, double ya, double xb, double yb)
{
  double dx = xb-xa; double dy = yb-ya;
  return sqrt(dx*dx + dy*dy);
}

int main(int argc, char** argv){
  	
		//myfile.open ("/home/bojan/Desktop/WC-Run/Speed/Hector/test-svn1.txt");
    		//myfile << "speed" << " 		" << "speed_theta" << "\n"; 
		ros::init(argc, argv, "my_tf_listener");	
		ros::NodeHandle node;

		tf::TransformListener listener;
  		ros::Rate rate(1);
		ros::Duration wait(5); wait.sleep();

		float last_time = ros::Time::now().toSec();int seq = 0;

		int speed_count = 0; double vel_linear, vel_angular, xlast = 0 ,ylast = 0, yaw_last = 0;
		
		while (node.ok())
		{

			//Speed calculation
			tf::StampedTransform transform;
    			try{
      				listener.lookupTransform("/map", "/odom",  
                               	ros::Time(0), transform);
			
    				}
    				catch (tf::TransformException ex){};

    				vel_linear = hypotenuse(xlast,ylast, transform.getOrigin().x(),transform.getOrigin().y());
				vel_angular = fabs(tf::getYaw(transform.getRotation()) - yaw_last);
			

			speed_count = 0;
			xlast = transform.getOrigin().x(); ylast = transform.getOrigin().y();
			yaw_last = tf::getYaw(transform.getRotation());
			ROS_INFO("linear speed= %f, anglular speed=  %f", vel_linear, vel_angular*57.29); 		
			myfile << vel_linear <<  " 		" << vel_angular*57.29 << "\n";   

					//ROS_INFO("%d: dt = %f",seq, ros::Time::now().toSec()-last_time);
					/*last_time = ros::Time::now().toSec(); */seq++;

			rate.sleep();
		}

  return 0;
};


