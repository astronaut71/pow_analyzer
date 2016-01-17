#include <ros/ros.h>
#include <fstream>
#include <string>
#include "sensor_msgs/Imu.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/PoseStamped.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include "sensor_msgs/LaserScan.h"
#include "pow_analyzer/Num1.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <sstream>
#include <tf/transform_listener.h>
#include "std_msgs/Float64.h"



#define TURN_ANGLE 50 //degrees
const double thresh_ang = TURN_ANGLE * (3.1416/180.0);


std::ofstream myfile;
using namespace std;
nav_msgs::Odometry odom;
//ros::Publisher pub;

ros::Publisher gr_p; std_msgs::Float64MultiArray gr_out1;

double time_prev;
double time_last, dtv, xlast = 0, ylast = 0, yaw_last = 0,  mark=0,  delta_t, vel_linear, vel_angular, accel, curr_yaw;
double r,p=0,y, initial_yaw, dist_curr, start_turn_angle ,end_turn_angle, initial_x, initial_y; tf::Quaternion quat;
double dxv, dyv;
bool is_first = 1,end_pose_rec1=0;; bool is_first1 = 1, first_time_rec = 0, first_idle_time_rec = 0, end_time_rec = 0;
int iter = 0; bool counting = 0;
bool prev_state_save1=1;
bool first_pose_rec1=0;
bool first_pose_rec=0, end_pose_rec=0;
geometry_msgs::Pose start_pose, end_pose;
geometry_msgs::Pose start_pose1, end_pose1;
geometry_msgs::PoseStamped prev_pose1;
double cma_lin_v=0,sig_lin_v=0,std_dev_lin_v,ox, oy, lx, ly, dx,dy,dt,lin_v, cma_yaw=0,sig_yaw=0,std_dev_yaw;
double dx1,dy1;
int lin_v_c=0, yaw_c=0; bool prev_state_save=1; geometry_msgs::PoseStamped prev_pose;
ros::Time start_time, end_time, start_idle_time, end_idle_time, now;
std::vector<double> yaws,vels;

sensor_msgs::LaserScan laser_scan;
float min_range;
bool datum_time_rec = 0;

double first_time_imu,time_last_imu, timestamp;

double normalize(double in)
{
if(in < -M_PI) return in + 2*M_PI;
else if(in > M_PI) return in - 2*M_PI;
else return in;
}
//Added Idle Time, Av. Angular Vel, Std. Ang. Vel, End conditopn by recording end orientation by vel=0 and time more than 10s
void imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
  tf::quaternionMsgToTF(msg->orientation, quat);
  tf::Matrix3x3(quat).getRPY(r,p,y); 
 
double angle_test=y;


  if(!datum_time_rec) { 
	first_time_imu = time_last_imu = ros::Time::now().toSec();
	//first_time_imu = time_last_imu = ros::Time::now();
	datum_time_rec = 1;
  }

// Initial yaw record
  if(is_first)
  {
	initial_yaw = y; ROS_INFO("Datum yaw %f deg", y*57.3);
	// Idle time, start recording
	start_idle_time =ros::Time::now();ROS_INFO("Start idle time recorded");
	first_idle_time_rec = 1;
	is_first = 0;
  }
  
// Start turning 
if(fabs(y-initial_yaw) >= thresh_ang && !first_time_rec)
	{
	 end_idle_time = ros::Time::now(); ROS_INFO("End idle time recorded");	
		start_turn_angle = y;	
		start_time = ros::Time::now(); ROS_INFO("Start time recorded on angle %f",start_turn_angle*57.3);
		first_time_rec = 1;
	}
	
// End turning
if(y >= (start_turn_angle + 2.08109041))     
	{
		end_turn_angle = y; 
		ROS_INFO("End time recorded on angle %f", end_turn_angle*57.3);
		end_time = ros::Time::now();
		end_time_rec = 1;
	}

//Time
//timestamp = (ros::Time::now().toSec()-first_time_imu);  //ROS_INFO("AT TIME %f", timestamp);
//ROS_INFO("linear speed=  %f, anglular speed=  %f", vel_linear, vel_angular); 
myfile << timestamp << "		" << angle_test*57.3 << "\n";  
// End Time


//Standard deviation for the orientation
//if(!is_first && !first_time_rec)
if(!is_first && !end_time_rec)
  {
	cma_yaw = (y + yaw_c*cma_yaw) / (double)(yaw_c+1); yaw_c++;
	yaws.push_back(y);
 }
}

void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{

//start driving distance
 
 now = ros::Time::now();
 if(first_idle_time_rec && !first_pose_rec1) 
  //if(first_time_rec && !first_pose_rec)
  {
  //now = ros::Time::now(); original
  
	prev_pose1.pose = start_pose1 = msg->pose;
	prev_state_save1 = 0;
	start_time = ros::Time::now(); 
	prev_pose1.header.stamp = ros::Time::now();
	first_pose_rec1 = 1; ROS_INFO("Start drive pose recorded");
	 
	}
	

/* original
	dxv = msg->pose.position.x-prev_pose1.pose.position.x; 
	dyv = msg->pose.position.y-prev_pose1.pose.position.y;
	float dist1 = sqrt(dxv*dxv+dyv*dyv);
	//ROS_INFO("Dist = %f", dist1);
	//dt = (now-prev_pose.header.stamp).toSec();
	dtv = (now-prev_pose1.header.stamp).toSec();
	if(dtv>0.0)
	{
		lin_v = sqrt(dxv*dxv+dyv*dyv)/dtv;
		//ROS_INFO("Time = %f", dtv);
		cma_lin_v = (lin_v + lin_v_c*cma_lin_v) / (double)(lin_v_c+1); lin_v_c++;
		vels.push_back(lin_v);
		prev_pose.header.stamp = now; prev_pose.pose = msg->pose;
	
  }
  */
  
//STart turning
 if(first_time_rec && !first_pose_rec)
  {
	prev_pose.pose = start_pose = msg->pose;
	prev_state_save = 0;
	prev_pose.header.stamp = ros::Time::now();
	first_pose_rec = 1; ROS_INFO("Start turn pose recorded");
  }
//end driving distance


  if(first_time_rec && !end_pose_rec1)
  {
	end_pose1 = msg->pose;
	end_pose_rec1 = 1; ROS_INFO("End drive pose recorded");
  }

  if(end_time_rec && !end_pose_rec)
  {
	end_pose = msg->pose;
	end_pose_rec = 1; ROS_INFO("End pose recorded");
  }
 
	if(!prev_state_save1 && first_pose_rec1) prev_state_save1 = 1;
  if(prev_state_save1 && first_pose_rec1 && !first_time_rec)
  {
	now = ros::Time::now();
	dxv = msg->pose.position.x-prev_pose1.pose.position.x; 
	dyv = msg->pose.position.y-prev_pose1.pose.position.y;
	dt = (now-prev_pose.header.stamp).toSec();
	float dist1 = sqrt(dxv*dxv+dyv*dyv); // komentirano bese
	ROS_INFO("Dist = %f", dist1); // komentirano bese
	//dt = (now-prev_pose.header.stamp).toSec();
	dtv = (now-prev_pose1.header.stamp).toSec(); // komentirano bese

		if(dtv>0.0)
		
		{
		lin_v = sqrt(dxv*dxv+dyv*dyv)/dtv;
		cma_lin_v = (lin_v + lin_v_c*cma_lin_v) / (double)(lin_v_c+1); lin_v_c++;
		vels.push_back(lin_v);
		prev_pose.header.stamp = now; prev_pose.pose = msg->pose;
	}
	
	
	 gr_out1.data[0] = (now-start_time).toSec();
	 ROS_INFO("Vreme= %f", (now-start_time).toSec());
  gr_out1.data[1] = lin_v;
  gr_out1.data[2] = tf::getYaw(msg->pose.orientation);
  gr_out1.data[3] = dist1;
  gr_out1.data[4] = min_range;
  
  gr_p.publish(gr_out1);
  ROS_INFO("Lin Vel = %f", lin_v);
  //ROS_INFO("Min = %f", min_range);
  //ROS_INFO("Dist = %f", dist1);
  //ROS_INFO("Time = %f", (now-start_time).toSec());
  }
 }
//}


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

    min_range = 2;
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

  myfile.open ("/home/bojan/POW/Assessment/Pida/180degturn.txt");
  myfile << "timestamp" << "		" << "Angle yaw" << "\n"; 
  
  gr_p = nh.advertise<std_msgs::Float64MultiArray>("to_graph", 1); gr_out1.data.resize(4);
  ros::Publisher node_pub = nh.advertise<std_msgs::Float64MultiArray>("stats", 1);
  
 
   //Task parameter
  std::string pida_turn_180deg_asses;
  nh.param("pida_turn_180deg_asses", pida_turn_180deg_asses, std::string("180_deg_turn"));
  // Create a publisher and name the topic.
  ros::Publisher pub_task = nh.advertise<std_msgs::String>("to_task", 1);
 
	 //File parameter
  std::string turn180_file;
  nh.param("turn180_file", turn180_file, std::string("180turn_run1"));
  // Create a publisher and name the topic.
  ros::Publisher pub_file = nh.advertise<std_msgs::String>("to_file", 1);
 
 
while(!end_pose_rec && ros::ok()) ros::spinOnce();

		laser_pub.publish(laser_scan);
		dx = end_pose.position.x-start_pose.position.x;
  dy = end_pose.position.y-start_pose.position.y;
  
  dx1 = end_pose1.position.x-start_pose1.position.x;
  dy1 = end_pose1.position.y-start_pose1.position.y;

 		double dist_tot = sqrt(dx*dx+dy*dy), dist_tot1 = sqrt(dx1*dx1+dy1*dy1), time = (end_time-start_time).toSec(), idle_time=(end_idle_time-start_idle_time).toSec(), 
 		avg_ang_v=(y/time)*57.3, avg_lin_v = dist_tot/time;

	
  		for(size_t i=0; i<yaws.size(); i++) sig_yaw += pow(yaws[i]-cma_yaw, 2);
  		std_dev_yaw = sqrt(sig_yaw/yaw_c)*57.3;
  		for(size_t i=0; i<vels.size(); i++) sig_lin_v += pow(vels[i]-cma_lin_v, 2);
 		 std_dev_lin_v = sqrt(sig_lin_v/lin_v_c);

				std_msgs::Float64MultiArray msg_array; msg_array.data.clear();
 			msg_array.data.push_back(std_dev_yaw);
 			msg_array.data.push_back(std_dev_lin_v);
 			msg_array.data.push_back(avg_lin_v);
 			msg_array.data.push_back(dist_tot);
 			msg_array.data.push_back(dist_tot1);
 			msg_array.data.push_back(time);
 			msg_array.data.push_back(idle_time);
 			msg_array.data.push_back(min_range);
    node_pub.publish(msg_array);
 			ROS_INFO("Published run statistics");

 	 	//ROS_INFO("Std dev --- Orientation: %f, Linear speed: %f", std_dev_yaw, std_dev_lin_v);
  		//ROS_INFO("avarege linear velocity is %f, turn dist %f", avg_lin_v, dist_tot);
  		//ROS_INFO("Drived distance %f meter", dist_tot1);
  		 ROS_INFO("avarege angular velocity is %f ", avg_ang_v);
  		//ROS_INFO("Travelled %f seconds", time);
  		//ROS_INFO("Idle time %f seconds", idle_time);
		  //ROS_INFO("Minimal distance to any obstale %f meters", min_range);
		  
		  
		  ROS_INFO("Std dev --- Orientation: %f, Linear speed: %f", std_dev_yaw, std_dev_lin_v);
  		ROS_INFO("avarege linear velocity is %f, total dist %f", avg_lin_v, dist_tot);
  		ROS_INFO("Travelled %f seconds", time);
		  ROS_INFO("Minimal distance to any obstale %f meters", min_range);
		  
		  ROS_INFO("Drived distance %f meter", dist_tot1);
		  ROS_INFO("Idle time %f seconds", idle_time);
  
    //Task 
 		 std_msgs::String msg_task; 
 		 msg_task.data=pida_turn_180deg_asses.c_str();
 		 
 		 //File 
 		 std_msgs::String msg_file; 
 		 msg_file.data=turn180_file.c_str();
 		 
 		 
 		 //Publish
 		 node_pub.publish(msg_array);
 		 pub_task.publish(msg_task);
 			pub_file.publish(msg_file);
 			
 			ROS_INFO("Published run statistics");
 			//ROS_INFO("task running is  ", pida_turn_180deg_asses.c_str());
 			//ROS_INFO("file running is  ", turn180_file.c_str());


 ros::spin();
 return 0;
}

 			















