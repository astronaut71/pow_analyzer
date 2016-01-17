#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <string>
#include "sensor_msgs/Imu.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "pow_analyzer/Num.h"
#include "std_msgs/Float64MultiArray.h"



#define TURN_ANGLE 2 //degrees
const double thresh_ang = TURN_ANGLE * (3.1416/180.0);

nav_msgs::Odometry odom;

double time_last, xlast = 0, ylast = 0, yaw_last = 0, delta_t, vel_linear, vel_angular, accel, curr_yaw, start_turn_angle, end_park_angle;
double r,p,y=0, initial_yaw , temp_yaw,current_yaw, start_angle,end_angle, ox,oy,lx,ly, turned_angle, start_park_angle, initial_x, initial_y; tf::Quaternion quat;
bool is_first = 1; bool first_time_rec = 0, start_rec = 0, end_time_rec = 0;
int iter = 0; bool counting = 0;
std::ofstream myfile;
bool is_first1 = 1;

bool first_pose_rec=0, start_pose_rec=0, end_pose_rec=0;
geometry_msgs::Pose start_pose, end_pose;

double  cma_yaw=0,sig_yaw=0,std_dev_yaw,cma_lin_v=0,sig_lin_v=0,std_dev_lin_v,dx,dy,dt,lin_v, lin_v1,dx1,dy1,dt1;
int lin_v_c=0, yaw_c=0; bool prev_state_save=1;  geometry_msgs::PoseStamped prev_pose; geometry_msgs::PoseStamped prev_pose1;

bool prev_state_save1=0;
ros::Time start_time, end_time;
ros::Time now;
ros::Time now1;
std::vector<double> yaws,vels;

sensor_msgs::LaserScan laser_scan;
float min_range;

ros::Publisher gr_p; std_msgs::Float64MultiArray gr_out;

double normalize(double in)
{
if(in < -M_PI) return in + 2*M_PI;
else if(in > M_PI) return in - 2*M_PI;
else return in;
}

void imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
  tf::quaternionMsgToTF(msg->orientation, quat);
  tf::Matrix3x3(quat).getRPY(r,p,y); 
 
// Initial yaw record
  if(is_first)
  	{
	initial_yaw = y; ROS_INFO("Datum yaw %f deg", y*57.3);
	is_first = 0;
 	}

  if(first_pose_rec  && !first_time_rec)
  	{
	start_angle= y;
	ROS_INFO("start driving recorded on angle %f", start_angle*57.3);
	//end_time = ros::Time::now();
	first_time_rec = 1; 
  	}
 

 if(end_pose_rec && !end_time_rec)
  	{
	end_park_angle= y;
	ROS_INFO("End driving recorded on angle %f", end_park_angle*57.3);
	//end_time = ros::Time::now();
	end_time_rec = 1; 
  	}


 if(!is_first && !end_time_rec)
  {
	cma_yaw = (y + yaw_c*cma_yaw) / (double)(yaw_c+1); yaw_c++;
	yaws.push_back(y);
  }

}


void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
// Get the pose when start driving with velocity bigger than threshhold
  
	now = ros::Time::now(); 
	dx = msg->pose.position.x-prev_pose.pose.position.x;  
	//ROS_INFO("dx %f", dx);
	dy = msg->pose.position.y-prev_pose.pose.position.y;
	dt = (now-prev_pose.header.stamp).toSec();
	if(dt>0.0)
	{
		//new angular
	 //curr_yaw = normalize(tf::getYaw(msg->pose.orientation) - 0.5*M_PI);
	 //odom.pose.orientation = tf::createQuaternionMsgFromYaw(curr_yaw);
	 //
		lin_v = sqrt(dx*dx+dy*dy)/dt; 
		cma_lin_v = (lin_v + lin_v_c*cma_lin_v) / (double)(lin_v_c+1); lin_v_c++;
		vels.push_back(lin_v);
		prev_pose.header.stamp = now; prev_pose.pose = msg->pose;
		if(lin_v > 0.2 && !first_pose_rec)
			{	
			prev_pose.pose = start_pose = msg->pose;
			prev_state_save = 0;
			start_time = ros::Time::now();
			first_pose_rec = 1; ROS_INFO("start pose recorded");
  			}
	}
	
  if(!prev_state_save && first_pose_rec) prev_state_save = 1;
  if(prev_state_save && first_pose_rec && !end_pose_rec)
  {
	//ROS_INFO("MARK");
	now = ros::Time::now(); 
	dx1 = msg->pose.position.x-prev_pose1.pose.position.x; 
	dy1 = msg->pose.position.y-prev_pose1.pose.position.y;
	float dist1 = sqrt(dx1*dx1+dy1*dy1);

	//ROS_INFO("dy1 %f", dy1);
	//dt1 = (now-prev_pose1.header.stamp).toSec();

	if(dist1 >= 10 && !end_pose_rec)
			{

			end_pose = msg->pose; 
			end_time = ros::Time::now();  //ROS_INFO("time  %f", end_time )
			end_pose_rec = 1; ROS_INFO("End pose recorded");
  	}
 
  gr_out.data[0] = (now-start_time).toSec();
  gr_out.data[1] = lin_v;
  gr_out.data[2] = tf::getYaw(msg->pose.orientation);
  gr_out.data[3] = dist1;
  gr_out.data[4] = min_range;
  //ROS_INFO("Dist = %f", dist1);
  gr_p.publish(gr_out);
   }
}


void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

    std::vector<float> laser;
    laser = msg->ranges;

    int size_laser = laser.size();
    for (int i=0;i<size_laser;i++){
        if (laser[i] < 0.01){
            laser[i] = 99999;
        }
        if (laser[i] > 45){
            laser[i] = 99999;
        }
    }

    min_range = 3;
    int index_min;
    for (int i=0;i<size_laser;i++){
        if (laser[i] < min_range){
            min_range = laser[i];
            index_min = i;
	    //ROS_INFO("Minimum Range = %f", min_range);
        }
    }

    for (int j=0;j<size_laser;j++){
        if (laser[j] > min_range + 0.5){
            laser[j] = 0;
        }
    }



    laser_scan = *msg;
    laser_scan.ranges.clear();
    laser_scan.ranges = laser;

}

int main(int argc, char **argv)
{
ros::init(argc, argv, "talker");
ros::NodeHandle node;
  
  ros::NodeHandle nh;
  ros::Subscriber sub2 = nh.subscribe("slam_out_pose", 1, poseCallback);
  ros::Subscriber sub =  nh.subscribe("raw_imu_throttle", 1, imuCallback);
  ros::Subscriber sub3 =  nh.subscribe("scan_throttle", 1, scanCallback);
  ros::Publisher laser_pub = nh.advertise<sensor_msgs::LaserScan>("closest_points", 1);
  
  gr_p = nh.advertise<std_msgs::Float64MultiArray>("to_graph", 1); gr_out.data.resize(4);
  ros::Publisher node_pub = nh.advertise<std_msgs::Float64MultiArray>("stats", 1);
  
  //Task parameter
  std::string wst_forward10m_raw_assess;
  nh.param("wst_forward10m_raw_assess", wst_forward10m_raw_assess, std::string("10m_forward"));
  // Create a publisher and name the topic.
  ros::Publisher pub_task = nh.advertise<std_msgs::String>("to_task", 1);
 
	 //File parameter
  std::string forward10m_file;
  nh.param("forward10m_file", forward10m_file, std::string("10m_run1"));
  // Create a publisher and name the topic.
  ros::Publisher pub_file = nh.advertise<std_msgs::String>("to_file", 1);

  while(!end_pose_rec && ros::ok()) ros::spinOnce();
						 
		  laser_pub.publish(laser_scan);
		  //ROS_INFO("Minimum Range = %f", min_range);
	 	 dx = end_pose.position.x-start_pose.position.x;
  		dy = end_pose.position.y-start_pose.position.y;
 	 	double dist_tot = sqrt(dx*dx+dy*dy), time = (end_time-start_time).toSec(), avg_lin_v = dist_tot/time, avg_ang_v=abs((y/time)*57);

	
  		for(size_t i=0; i<yaws.size(); i++) sig_yaw += pow(yaws[i]-cma_yaw, 2);
  		std_dev_yaw = sqrt(sig_yaw/yaw_c)*57.3;
  		for(size_t i=0; i<vels.size(); i++) sig_lin_v += pow(vels[i]-cma_lin_v, 2);
 		 std_dev_lin_v = sqrt(sig_lin_v/lin_v_c);

 	 	ROS_INFO("Std dev --- Orientation: %f, Linear speed: %f", std_dev_yaw, std_dev_lin_v);
  		ROS_INFO("avarege linear velocity is %f, total dist %f", avg_lin_v, dist_tot);
  		ROS_INFO("Travelled %f seconds", time);
		  ROS_INFO("Minimal distance to the wall is %f", min_range);
		  //ROS_INFO("avarege angular velocity is %f ", avg_ang_v);
		  
 			
 			std_msgs::Float64MultiArray msg_array; msg_array.data.clear();
 			msg_array.data.push_back(std_dev_yaw);
 			msg_array.data.push_back(std_dev_lin_v);
 			msg_array.data.push_back(avg_lin_v);
 			msg_array.data.push_back(time);
 			msg_array.data.push_back(min_range);
 			//msg_array.data.push_back(avg_ang_v);
 			 
 			//Task 
 		 std_msgs::String msg_task; 
 		 msg_task.data=wst_forward10m_raw_assess.c_str();
 		 
 		 //File 
 		 std_msgs::String msg_file; 
 		 msg_file.data=forward10m_file.c_str();
 			
 			//Publish
 			node_pub.publish(msg_array);
 			pub_task.publish(msg_task);
 			pub_file.publish(msg_file);
 			
 			ROS_INFO("Published run statistics");
 			ROS_INFO("task running is  ", wst_forward10m_raw_assess.c_str());
 			ROS_INFO("file running is  ", forward10m_file.c_str());
 			
 return 0;
}


















