#include <ros/ros.h>
#include <fstream>
#include "sensor_msgs/Imu.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"
//ramp angle threshold
#define RAMP_ANGLE 3.65//degrees

double time_prev;
double time_last, xlast = 0, ylast = 0, yaw_last = 0,  mark=0,  delta_t, vel_linear, vel_angular, accel, curr_yaw;
const double ramp_ang = RAMP_ANGLE * (3.1416/180.0);
double r,p=0,y, initial_yaw ,initial_pitch,ramp_pitch, initial_x, initial_y, temp_pitch; tf::Quaternion quat;
ros::Time start_time, end_time;
bool is_first = 1; bool is_first1 = 1, first_time_rec = 0, end_time_rec = 0;
int iter = 0; bool counting = 0;
std::ofstream myfile;

bool first_pose_rec=0, end_pose_rec=0;
geometry_msgs::Pose start_pose, end_pose;

double cma_lin_v=0,sig_lin_v=0,std_dev_lin_v,dx,dy,dt,lin_v, cma_yaw=0,sig_yaw=0,std_dev_yaw;
int lin_v_c=0, yaw_c=0; bool prev_state_save=1; geometry_msgs::PoseStamped prev_pose;
ros::Time now;
std::vector<double> yaws,vels;

sensor_msgs::LaserScan laser_scan;
float min_range;

void imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
  tf::quaternionMsgToTF(msg->orientation, quat);
  tf::Matrix3x3(quat).getRPY(r,p,y); 
 
// Initial pitch record
  if(is_first)
  {
	initial_pitch = p; ROS_INFO("Datum pitch %f deg", p*57.3);
	initial_yaw = y; ROS_INFO("Datum yaw %f deg", y*57.3);
	is_first = 0;
  }
  else if(counting)
  {
	if(iter<1) iter++;
	else if(iter==1)
	{
		if(fabs((p-initial_pitch)-temp_pitch)*57.3 < 0.4)
		{
			end_time_rec = 1; counting = 0;
			end_time = ros::Time::now(); ROS_INFO("End time recorded on angle %f", temp_pitch*57.3);
		}
		else iter = counting = 0;
	}
  }  
  else
  {
	if(fabs(p-initial_pitch) >= ramp_ang && !first_time_rec)
	{
		ramp_pitch = fabs(p-initial_pitch)*57.3;	
		start_time = ros::Time::now(); ROS_INFO("Start time recorded on ramp angle %f", ramp_pitch);
		first_time_rec = 1;
	}
	else if((first_time_rec && !end_time_rec) && fabs((p-initial_pitch)-ramp_pitch/57.3) >= ramp_ang)
	{
		//ROS_INFO("Potential end of ramp recorded");
		end_time = ros::Time::now();
		counting = 1; temp_pitch = (p-initial_pitch);
	}
  }
  if(!is_first && !end_time_rec)
  {
	cma_yaw = (y + yaw_c*cma_yaw) / (double)(yaw_c+1); yaw_c++;
	yaws.push_back(y);
  }
}

void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  if(first_time_rec && !first_pose_rec)
  {
	prev_pose.pose = start_pose = msg->pose;
	prev_state_save = 0;
	prev_pose.header.stamp = ros::Time::now();
	first_pose_rec = 1; ROS_INFO("Start pose recorded");
  }
  if(end_time_rec && !end_pose_rec)
  {
	end_pose = msg->pose;
	end_pose_rec = 1; ROS_INFO("End pose recorded");
  }

  if(!prev_state_save && first_pose_rec) prev_state_save = 1;
  if(prev_state_save && first_pose_rec && !end_time_rec)
  {
	now = ros::Time::now();
	dx = msg->pose.position.x-prev_pose.pose.position.x;  
	dy = msg->pose.position.y-prev_pose.pose.position.y;
	dt = (now-prev_pose.header.stamp).toSec();
	if(dt>0.0)
	{
		lin_v = sqrt(dx*dx+dy*dy)/dt;
		cma_lin_v = (lin_v + lin_v_c*cma_lin_v) / (double)(lin_v_c+1); lin_v_c++;
		vels.push_back(lin_v);

		prev_pose.header.stamp = now; prev_pose.pose = msg->pose;
	}
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
  ros::init(argc, argv, "imu_odom");
  ros::NodeHandle nh;
  ros::Subscriber sub2 = nh.subscribe("slam_out_pose", 1, poseCallback);
  ros::Subscriber sub =  nh.subscribe("raw_imu", 1, imuCallback);
  ros::Subscriber sub3 =  nh.subscribe("scan", 1, scanCallback);
  ros::Publisher laser_pub = nh.advertise<sensor_msgs::LaserScan>("closest_points", 1);

  while(!end_pose_rec && ros::ok()) ros::spinOnce();
  
  myfile.open("/home/bojan/svn/POW/ROS/Fuerte/Data/Assessment/Ramp_Down/up16.txt");

  dx = end_pose.position.x-start_pose.position.x;
  dy = end_pose.position.y-start_pose.position.y;
  double dist_tot = sqrt(dx*dx+dy*dy), time = (end_time-start_time).toSec(), avg_lin_v = dist_tot/time;

  for(size_t i=0; i<yaws.size(); i++) sig_yaw += pow(yaws[i]-cma_yaw, 2);
  std_dev_yaw = sqrt(sig_yaw/yaw_c)*57.3;
  for(size_t i=0; i<vels.size(); i++) sig_lin_v += pow(vels[i]-cma_lin_v, 2);
  std_dev_lin_v = sqrt(sig_lin_v/lin_v_c);

  ROS_INFO("Std dev --- Orientation: %f, Linear speed: %f", std_dev_yaw, std_dev_lin_v);
  ROS_INFO("avarege linear velocity is %f, total dist %f", avg_lin_v, dist_tot);
  ROS_INFO("Travelled ramp of average %f degrees, in %f seconds", ramp_pitch, time);
  ROS_INFO("Minimal distance to the wall is %f", min_range);
  myfile << ramp_pitch <<  "	" << std_dev_yaw <<  "	" << std_dev_lin_v << "	" << avg_lin_v <<  "	" << dist_tot << "	" << time <<	"	" << min_range <<	"\n";

ros::spin();
 return 0;
}
