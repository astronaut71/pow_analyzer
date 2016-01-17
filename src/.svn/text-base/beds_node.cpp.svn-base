#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Quaternion.h"
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <fstream>

std::ofstream myfile;
using namespace std;
visualization_msgs::Marker line;
visualization_msgs::Marker cube1;
visualization_msgs::Marker cube2;
visualization_msgs::Marker cube3;
visualization_msgs::Marker cube4;
ros::Publisher marker_pub;
ros::Publisher marker_pub1;
ros::Publisher marker_pub2;
ros::Publisher marker_pub3;
ros::Publisher marker_pub4;

double hypotenuse(double xa, double ya, double xb, double yb)
{
  double dx = xb-xa; double dy = yb-ya;
  return sqrt(dx*dx + dy*dy);
}

// return minimum distance between two line segments defined by 4 points
void DistanceFromSegment(double p1x, double p1y, double p2x, double p2y ,
			 double p3x, double p3y, double p4x, double p4y,double &distanceSegment)

{
    double EPS = 0.00000001;

    double ux = p1x - p2x;
    double uy = p1y - p2y;
    double vx = p3x - p4x;
    double vy = p3y - p4y;
    double wx = p2x - p4x;
    double wy = p2y - p4y;

    double a= ux*ux + uy*uy;
    double b= ux*vx + uy*vy;
    double c= vx*vx + vy*vy;
    double d= ux*wx + uy*wy;
    double e= vx*wx + vy*wy;

    double D = a*c - b*b;
    double sD = D;
    double tD = D;

    double sc;
    double sN;
    double tN;
    double tc;

    if (D < EPS)
    {
        sN = 0.0;
        sD = 1.0;
        tN = e;
        tD = c;
    }
    else
    {
        sN = (b * e - c * d);
        tN = (a * e - b * d);
        if (sN < 0.0)
        {
            sN = 0.0;
            tN = e;
            tD = c;
        }
        else if (sN > sD)
        {
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }

    if (tN < 0.0)
    {
        tN = 0.0;

        if (-d < 0.0)
            sN = 0.0;
        else if (-d > a)
            sN = sD;
        else
        {
            sN = -d;
            sD = a;
        }
    }
    else if (tN > tD)
    {
        tN = tD;
        if ((-d + b) < 0.0)
            sN = 0;
        else if ((-d + b) > a)
            sN = sD;
        else
        {
            sN = (-d + b);
            sD = a;
        }
    }

    if (fabs(sN) < EPS) {
        sc = 0.0;
                        }
        else { sc = sN / sD;
                }

        if (fabs(tN) < EPS){
            tc = 0.0;
                    }
            else  {
                tc = tN / tD;
               }

    	double   dPx = wx + (sc * ux) - (tc * vx);  // = S1(sc) - S2(tc)
    	double   dPy = wy + (sc * uy) - (tc * vy);  // = S1(sc) - S2(tc)

distanceSegment = sqrt(dPx*dPx + dPy*dPy);

return;
}

int main(int argc, char** argv){
  	
		//myfile.open ("/home/bojan/svn/POW/ROS/Fuerte/Data/Beds/User1/Final_pow_freek_3_2011-04-01-14-21-48.bag.txt");
    	//	myfile << "Yaw in deg" << "	" << "Velocity in m/s" << "	" << "Distance to bed in m" <<  "\n";
		ros::init(argc, argv, "my_tf_listener");	
		ros::NodeHandle node;

			//Visualize the BED
			marker_pub = node.advertise<visualization_msgs::Marker>("BED", 10);
			line.header.frame_id = "map";
	    		line.header.stamp = ros::Time::now();
	    		line.ns = "BED";
	    		line.type = visualization_msgs::Marker::LINE_STRIP;
	    		line.action = visualization_msgs::Marker::ADD;
	    		line.id = 0;

			    line.color.b = 1.0;
	    		line.color.a = 1.0;
	    		line.scale.x = 0.1;	//Sets the width of the LINE_STRIP
	   		 line.scale.y = 0.1;	//Ignored if marker type is LINE_STRIP
			    line.points.resize(2);
			


			//Visualize the PYLON1
			//marker_pub1 = node.advertise<visualization_msgs::Marker>("PYLON", 10);
			//cube1.header.frame_id = "map";
	    		//cube1.header.stamp = ros::Time::now();
	    		//cube1.ns = "PYLON";
	    		//cube1.type = visualization_msgs::Marker::CUBE;
	    		//cube1.action = visualization_msgs::Marker::ADD;
	    		//cube1.id = 1;

			//cube1.color.r = 1.0;
	    		//cube1.color.a = 1.0;
	    		//cube1.scale.x = 0.3;	//Sets the width of the LINE_STRIP
	   		//cube1.scale.y = 0.3;	//Ignored if marker type is LINE_STRIP
			//cube1.scale.z = 0.3;
		//	cube1.points.resize(2);
			

			//Visualize the PYLON2
			/*
			marker_pub2 = node.advertise<visualization_msgs::Marker>("PYLON", 10);
			cube2.header.frame_id = "map";
	    		cube2.header.stamp = ros::Time::now();
	    		cube2.ns = "PYLON";
	    		cube2.type = visualization_msgs::Marker::CUBE;
	    		cube2.action = visualization_msgs::Marker::ADD;
	    		cube2.id = 2;

			cube2.color.r = 1.0;
	    		cube2.color.a = 1.0;
	    		cube2.scale.x = 0.3;	//Sets the width of the LINE_STRIP
	   		cube2.scale.y = 0.3;	//Ignored if marker type is LINE_STRIP
			cube2.scale.z = 0.3;
			cube2.points.resize(2);

			//Visualize the PYLON3
			marker_pub3 = node.advertise<visualization_msgs::Marker>("PYLON", 10);
			cube3.header.frame_id = "map";
	    		cube3.header.stamp = ros::Time::now();
	    		cube3.ns = "PYLON";
	    		cube3.type = visualization_msgs::Marker::CUBE;
	    		cube3.action = visualization_msgs::Marker::ADD;
	    		cube3.id = 3;

			cube3.color.r = 1.0;
	    		cube3.color.a = 1.0;
	    		cube3.scale.x = 0.3;	//Sets the width of the LINE_STRIP
	   		cube3.scale.y = 0.3;	//Ignored if marker type is LINE_STRIP
			cube3.scale.z = 0.3;
			cube3.points.resize(2);

			//Visualize the PYLON4
			marker_pub4 = node.advertise<visualization_msgs::Marker>("PYLON", 10);
			cube4.header.frame_id = "map";
	    		cube4.header.stamp = ros::Time::now();
	    		cube4.ns = "PYLON";
	    		cube4.type = visualization_msgs::Marker::CUBE;
	    		cube4.action = visualization_msgs::Marker::ADD;
	    		cube4.id = 4;

			cube4.color.r = 1.0;
	    		cube4.color.a = 1.0;
	    		cube4.scale.x = 0.3;	//Sets the width of the LINE_STRIP
	   		cube4.scale.y = 0.3;	//Ignored if marker type is LINE_STRIP
			cube4.scale.z = 0.3;
			cube4.points.resize(2);
		*/

		tf::TransformListener listener;
  		ros::Rate rate(10.0);
		ros::Duration wait(5); wait.sleep();

		int speed_count = 0; double vel,yaw, xlast = 0 ,ylast = 0;
		
		while (node.ok())
			{

			//Speed calculation
			tf::StampedTransform transform2;
		if(speed_count==9)
		{
    			try{
      				//listener.lookupTransform("/map", "/scanmatcher_frame",  
      				listener.lookupTransform("/map", "/base_link", 
          ros::Time(0), transform2);
			
    			}
    				catch (tf::TransformException ex){};

    			vel = hypotenuse(xlast,ylast, transform2.getOrigin().x(),transform2.getOrigin().y());
			if(vel<0.003){
				yaw = tf::getYaw(transform2.getRotation());
			}

			speed_count = 0;
			xlast = transform2.getOrigin().x(); ylast = transform2.getOrigin().y();
		}
		else speed_count++;
				//Get the 4 corners of the wheelchair		

			tf::StampedTransform transform;
    			try{
      				listener.lookupTransform("/map", "/corner1",  
                               	ros::Time(0), transform);
			
    			}
    				catch (tf::TransformException ex){};
    			
			double x_vect_a = transform.getOrigin().x();
			double y_vect_a = transform.getOrigin().y();
			
    			
			try{
      				listener.lookupTransform("/map", "/corner2",  
                               	ros::Time(0), transform);
    			}
    				catch (tf::TransformException ex){};
    			
			double x_vect_b = transform.getOrigin().x();
			double y_vect_b = transform.getOrigin().y();
			
			try{
      				listener.lookupTransform("/map", "/corner3",  
                               	ros::Time(0), transform);
    			}
    				catch (tf::TransformException ex){};
    			
			double x_vect_c = transform.getOrigin().x();
			double y_vect_c = transform.getOrigin().y();

			try{
      				listener.lookupTransform("/map", "/corner4",  
                               	ros::Time(0), transform);
    			}
    				catch (tf::TransformException ex){};
    			
			double x_vect_d = transform.getOrigin().x();
			double y_vect_d = transform.getOrigin().y();

			//Find the minimum distance between the four line segments of the rectangle(wheelchair) and bed frame (line segment)
		
			//Line segment 1, between corner1 and corner2			
			//double p1x = -9.5; 
			//double p1y =  4.2;
			//double p2x = -9.5;
			//double p2y =  2.5;
			//hector
			double p1x = -3.3; 
			double p1y = -12.2;
			double p2x = -1.3;
			double p2y =  -11.75;			
			
			

			double p3x =  x_vect_a;
			double p3y =  y_vect_a;
			double p4x =  x_vect_b;
			double p4y =  y_vect_b;
			double distanceSegment1; 
			DistanceFromSegment(p1x, p1y, p2x, p2y ,p3x, p3y, p4x, p4y, distanceSegment1);
			
			//Line segment 2, between corner2 and corner3		
			double distanceSegment2; 
			DistanceFromSegment(p1x, p1y, p2x, p2y ,x_vect_b, y_vect_b, x_vect_c, y_vect_c, distanceSegment2);

			//Line segment 3, between corner3 and corner4			
			double distanceSegment3; 
			DistanceFromSegment(p1x, p1y, p2x, p2y ,x_vect_c, y_vect_c, x_vect_d, y_vect_d, distanceSegment3);
			
			//Line segment 4, between corner4 and corner1			
			double distanceSegment4; 
			DistanceFromSegment(p1x, p1y, p2x, p2y ,x_vect_d, y_vect_d, x_vect_a, y_vect_a, distanceSegment4);
			
			//Find the minimum distance of all 8 line to segment distances
			double distarray [4]= {distanceSegment1, distanceSegment2, distanceSegment3, distanceSegment4};
			
			double mindistance = distarray [0];
			for (int i = 1; i < 4; i++){
				if (distarray[i] < mindistance){
				    mindistance = distarray[i];
					 
								}
							}
					
			line.points[1].x = -3.3; line.points[1].y = -12.2;
			line.points[0].x = -1.3; line.points[0].y = -11.75; line.points[0].z = line.points[1].z = 0;
			//marker_pub.publish(line);
			
			
			//line.points[1].x = -3.5; line.points[1].y = -12.1;
			//line.points[0].x = -1.6; line.points[0].y = -11.7; line.points[0].z = line.points[1].z = 0;
			marker_pub.publish(line);
			
			
			//line.points[1].x = -8.7; line.points[1].y = 3.2;
			//line.points[0].x = -8.6; line.points[0].y = 1.4; line.points[0].z = line.points[1].z = 0;
			//marker_pub.publish(line);
			

			///PYLON
			//sphere.points[3].x = 0; cube.points[3].y = 0.5;
			//cube.points[2].x = 0.5; cube.points[2].y = 0.5; cube.points[0].z = cube.points[1].z = 0;
			//cube1.pose.position.x = -2.0; cube1.pose.position.y= -2.1;  cube1.pose.position.z = 1;

			//sphere.points[0].x = -3; sphere.points[0].y = 5; sphere.points[0].z = 5;
			/*
			sphere.pose.position.x = 1;
			sphere.pose.position.y = 1;
			sphere.pose.position.z = 0;
			sphere.pose.orientation.x = 0.0;
			sphere.pose.orientation.y = 0.0;
			sphere.pose.orientation.z = 0.0;
			sphere.pose.orientation.w = 0.0;
			*/

			
			//cube1.pose.position.x = -2.0; cube1.pose.position.y=  -2.1;  cube1.pose.position.z = 1;
		//	cube2.pose.position.x = -2.4; cube2.pose.position.y = -0.1;  cube2.pose.position.z = 1;
			//cube3.pose.position.x = -2.8; cube3.pose.position.y =  1.9;  cube3.pose.position.z = 1;
		//	cube4.pose.position.x = -3.2; cube4.pose.position.y =  3.9;  cube4.pose.position.z = 1;
		
			
			// U1R1
			/*
			cube1.pose.position.x = 1.5; cube1.pose.position.y = 0.9;  cube1.pose.position.z = 1;
			cube2.pose.position.x = 3.5; cube2.pose.position.y = 1.2;  cube2.pose.position.z = 1;
			cube3.pose.position.x = 5.5; cube3.pose.position.y = 1.5;  cube3.pose.position.z = 1;
			cube4.pose.position.x = 7.5; cube4.pose.position.y = 1.8;  cube4.pose.position.z = 1;	
			*/

			//U1R2
			/*
			cube1.pose.position.x = 1.1; cube1.pose.position.y = 1.1;  cube1.pose.position.z = 1;
			cube2.pose.position.x = 3.1; cube2.pose.position.y = 1.2;  cube2.pose.position.z = 1;
			cube3.pose.position.x = 5.1; cube3.pose.position.y = 1.3;  cube3.pose.position.z = 1;
			cube4.pose.position.x = 7.1; cube4.pose.position.y = 1.4;  cube4.pose.position.z = 1;
			*/			
			// U3R1
			/*
			cube1.pose.position.x = 1.9; cube1.pose.position.y = 1.1;  cube1.pose.position.z = 1;
			cube2.pose.position.x = 3.5; cube2.pose.position.y = 1.2;  cube2.pose.position.z = 1;
			cube3.pose.position.x = 5.3; cube3.pose.position.y = 1.3;  cube3.pose.position.z = 1;
			cube4.pose.position.x = 7.0; cube4.pose.position.y = 1.4;  cube4.pose.position.z = 1;	
			*/

			//marker_pub1.publish(cube1);
			//marker_pub2.publish(cube2);
			//marker_pub2.publish(cube3);
			//marker_pub2.publish(cube4);
					
					if(vel<0.0025){
					if (mindistance < 2) {
				ROS_INFO("Yaw: %f  Distance: %f",yaw*57.29 ,mindistance);
				//myfile << yaw*57.29 << " 	  " << vel <<  "        "  << mindistance <<  "\n";           
						    }
							}
				
			rate.sleep();
			}

  return 0;
};

