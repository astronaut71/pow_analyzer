#include <ros/ros.h>
#include <tf/transform_listener.h>
//#include "geometry_msgs/PoseWithCovarianceStamped.h"
//#include "geometry_msgs/Quaternion.h"
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <fstream>

std::ofstream myfile;
using namespace std;
visualization_msgs::Marker line;
ros::Publisher marker_pub;

// return minimum distance between line segment ab and point c
void DistanceFromLine(double cx, double cy, double ax, double ay ,
		  double bx, double by, double &distanceSegment,  double &distanceLine)
{

	double r_numerator = (cx-ax)*(bx-ax) + (cy-ay)*(by-ay);
	double r_denomenator = (bx-ax)*(bx-ax) + (by-ay)*(by-ay);
	double r = r_numerator / r_denomenator;
    	double px = ax + r*(bx-ax);
    	double py = ay + r*(by-ay);
    	double s =  ((ay-cy)*(bx-ax)-(ax-cx)*(by-ay) ) / r_denomenator;

	distanceLine = fabs(s)*sqrt(r_denomenator);

	double xx = px;
	double yy = py;

	if ( (r >= 0) && (r <= 1) )
	{
		distanceSegment = distanceLine;
	}
	else
	{

		double dist1 = (cx-ax)*(cx-ax) + (cy-ay)*(cy-ay);
		double dist2 = (cx-bx)*(cx-bx) + (cy-by)*(cy-by);
		if (dist1 < dist2)
		{
			xx = ax;
			yy = ay;
			distanceSegment = sqrt(dist1);
		}
		else
		{
			xx = bx;
			yy = by;
			distanceSegment = sqrt(dist2);
		}


	}
	line.points[1].x = xx; line.points[1].y = yy;
	marker_pub.publish(line);
	return;
}

int main(int argc, char** argv){
  
		
		myfile.open ("/home/bojan/svn/POW/ROS/Fuerte/Data/Doors/User1/Doors2_Min_Total_pow_freek_3_2011-04-01-14-21-48.txt");
    		myfile << "mindistance in m" << "\n";

		ros::init(argc, argv, "my_tf_listener");	
		ros::NodeHandle node;

			marker_pub = node.advertise<visualization_msgs::Marker>("line_to_door", 10);
			line.header.frame_id = "map";
	    		line.header.stamp = ros::Time::now();
	    		line.ns = "line_to_door";
	    		line.type = visualization_msgs::Marker::LINE_STRIP;
	    		line.action = visualization_msgs::Marker::ADD;
	    		line.id = 0;

			line.color.b = 1.0;
	    		line.color.a = 1.0;
	    		line.scale.x = 0.1;	//Sets the width of the LINE_STRIP
	   		line.scale.y = 0.1;	//Ignored if marker type is LINE_STRIP
			line.points.resize(2);
		
		tf::TransformListener listener;
  		ros::Rate rate(10.0);
		ros::Duration wait(5); wait.sleep();
		
		double total_min_distance = 100000.0;

		while (node.ok())
			{
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


			//Find the minimum distance between the four line segments of the rectangle and two doors frames of Door1
			//Door 1
			
			//Line segment 1, between corner1 and corner2			
			//Door1 frame 1
		 //double d1_1_x = -4.6; 
			//double d1_1_y =  5.3;
			// hector mapping
			double d1_1_x = -5.1; 
			double d1_1_y = -7.5;
			
			double distanceSegmentD1_11; 
			double distanceLineD1_11;
			
			double ax =  x_vect_a;
			double ay =  y_vect_a;
			double bx =  x_vect_b;
			double by =  y_vect_b;
			DistanceFromLine( d1_1_x,  d1_1_y,  ax,  ay ,bx,  by, distanceSegmentD1_11, distanceLineD1_11);
						
			//Door1 frame 2			
			//double d1_2_x = -4.6;
			//double d1_2_y =  4.0;
			// hector mapping
			double d1_2_x = -4.1; 
			double d1_2_y = -7.4;

			double distanceSegmentD1_12; 
			double distanceLineD1_12;	
			DistanceFromLine( d1_2_x, d1_2_y,  ax,  ay ,bx,  by, distanceSegmentD1_12, distanceLineD1_12);
			
			//Line segment 2, between corner2 and corner3			
			//Door1 frame 1
			
			double distanceSegmentD1_21; 
			double distanceLineD1_21;
			DistanceFromLine( d1_1_x,  d1_1_y,  x_vect_b,  y_vect_b ,x_vect_c,  y_vect_c, distanceSegmentD1_21, distanceLineD1_21);

			//Door1 frame 2			
			double distanceSegmentD1_22; 
			double distanceLineD1_22;
			DistanceFromLine( d1_2_x,  d1_2_y,  x_vect_b,  y_vect_b ,x_vect_c,  y_vect_c, distanceSegmentD1_22, distanceLineD1_22);

			//Line segment 3, between corner3 and corner4			
			//Door1 frame 1
			
			double distanceSegmentD1_31; 
			double distanceLineD1_31;
			DistanceFromLine( d1_1_x,  d1_1_y,  x_vect_c,  y_vect_c ,x_vect_d,  y_vect_d, distanceSegmentD1_31, distanceLineD1_31);
			
			//Door1 frame 2			
			double distanceSegmentD1_32;
			double distanceLineD1_32;
			DistanceFromLine( d1_2_x,  d1_2_y,  x_vect_c,  y_vect_c ,x_vect_d,  y_vect_d, distanceSegmentD1_32, distanceLineD1_32);

			//Line segment 4, between corner4 and corner1			
			//Door1 frame 1
			double distanceSegmentD1_41; 
			double distanceLineD1_41;
			DistanceFromLine( d1_1_x,  d1_1_y,  x_vect_d,  y_vect_d , x_vect_a,  y_vect_a, distanceSegmentD1_41, distanceLineD1_41);
			
			//Door1 frame 2			
			double distanceSegmentD1_42; 
			double distanceLineD1_42;
			DistanceFromLine( d1_2_x,  d1_2_y,  x_vect_d,  y_vect_d , x_vect_a,  y_vect_a, distanceSegmentD1_42, distanceLineD1_42);

			
			//Find the minimum distance between the four line segments of the rectangle and two doors frames of Door2			
			//Door2
			
			//Line segment 1, between corner1 and corner2			
			//Door2 frame 1
			//double d2_1_x = -4.7; 
			//double d2_1_y = -0.75;
			// hector mapping
			double d2_1_x = 1.8; 
			double d2_1_y = -6.25;

			double distanceSegmentD2_11; 
			double distanceLineD2_11;
			
			//double ax =  x_vect_a;
			//double ay =  y_vect_a;
			//double bx =  x_vect_b;
			//double by =  y_vect_b;
			DistanceFromLine( d2_1_x,  d2_1_y,  ax,  ay ,bx,  by, distanceSegmentD2_11, distanceLineD2_11);
						
			//Door frame 2			
			//double d2_2_x = -4.7;
			//double d2_2_y = -1.8;
			//hector mapping
			double d2_2_x = 0.8; 
			double d2_2_y = -6.5;

			double distanceSegmentD2_12; 
			double distanceLineD2_12;	
			DistanceFromLine( d2_2_x, d2_2_y,  ax,  ay ,bx,  by, distanceSegmentD2_12, distanceLineD2_12);
			
			//Line segment 2, between corner2 and corner3			
			//Door frame 1
			
			double distanceSegmentD2_21; 
			double distanceLineD2_21;
			DistanceFromLine( d2_1_x,  d2_1_y,  x_vect_b,  y_vect_b ,x_vect_c,  y_vect_c, distanceSegmentD2_21, distanceLineD2_21);

			//Door frame 2			
			double distanceSegmentD2_22; 
			double distanceLineD2_22;
			DistanceFromLine( d2_2_x,  d2_2_y,  x_vect_b,  y_vect_b ,x_vect_c,  y_vect_c, distanceSegmentD2_22, distanceLineD2_22);

			//Line segment 3, between corner3 and corner4			
			//Door frame 1
			
			double distanceSegmentD2_31; 
			double distanceLineD2_31;
			DistanceFromLine( d2_1_x,  d2_1_y,  x_vect_c,  y_vect_c ,x_vect_d,  y_vect_d, distanceSegmentD2_31, distanceLineD2_31);
			
			//Door frame 2			
			double distanceSegmentD2_32;
			double distanceLineD2_32;
			DistanceFromLine( d2_2_x,  d2_2_y,  x_vect_c,  y_vect_c ,x_vect_d,  y_vect_d, distanceSegmentD2_32, distanceLineD2_32);

			//Line segment 4, between corner4 and corner1			
			//Door frame 1
	
			double distanceSegmentD2_41; 
			double distanceLineD2_41;
			DistanceFromLine( d2_1_x,  d2_1_y,  x_vect_d,  y_vect_d , x_vect_a,  y_vect_a, distanceSegmentD2_41, distanceLineD2_41);
			
			//Door frame 2			
			double distanceSegmentD2_42; 
			double distanceLineD2_42;

			DistanceFromLine( d2_2_x,  d2_2_y,  x_vect_d,  y_vect_d , x_vect_a,  y_vect_a, distanceSegmentD2_42, distanceLineD2_42);



			//Store the minimum distance of all 16 line to segment distances in an array
			double distarray [16]= {distanceSegmentD1_11, distanceSegmentD2_11, distanceSegmentD1_12, distanceSegmentD2_12, distanceSegmentD1_21, 								  distanceSegmentD2_21, distanceSegmentD1_22, distanceSegmentD2_22, distanceSegmentD1_31, distanceSegmentD2_31,distanceSegmentD1_32, distanceSegmentD2_32, distanceSegmentD1_41, distanceSegmentD2_41, distanceSegmentD1_42, distanceSegmentD2_42};
			
			
			int count=0;
			double mindistance = distarray [0];
			for (int i = 1; i < 16; i++){
				if (distarray[i] < mindistance){
				    mindistance = distarray[i];
					 count=i+1;
								}
							}
						
											
						//double mindistance_array [1] = {mindistance};
						

						//Marker drawing
						//Door1 frame 1
								
					if (count == 1){
						DistanceFromLine( d1_1_x,  d1_1_y,  ax,  ay ,bx,  by, distanceSegmentD1_11, distanceLineD1_11);

						//double d1_1_x = -4.6; 
					 //double d1_1_y =  5.3;
						//hector mapping
						double d1_1_x = -5.1; 
						double d1_1_y = -7.5;
						double ax =  x_vect_a;
						double ay =  y_vect_a;
						double bx =  x_vect_b;
						double by =  y_vect_b;
						double r_numerator = (d1_1_x-ax)*(bx-ax) + (d1_1_y-ay)*(by-ay);
						double r_denomenator = (bx-ax)*(bx-ax) + (by-ay)*(by-ay);
						double r = r_numerator / r_denomenator;
					    	double px = ax + r*(bx-ax);
					    	double py = ay + r*(by-ay);
						double xx = px;
						double yy = py;
					
						line.points[1].x = xx; line.points[1].y = yy;						
						//line.points[0].x = -4.6; line.points[0].y = 5.3; 
						//hector
						line.points[0].x = -5.1; line.points[0].y = -7.5;
						line.points[0].z = line.points[1].z = 0;
						
							}

						//Door2 frame 1
					if (count == 2){
						DistanceFromLine( d2_1_x,  d2_1_y,  ax,  ay ,bx,  by, distanceSegmentD2_11, distanceLineD2_11);

						//double d2_1_x = -4.7; 
						//double d2_1_y = -0.75;
						//hector mapping
						double d2_1_x = 1.8; 
						double d2_1_y = -6.25;
						double ax =  x_vect_a;
						double ay =  y_vect_a;
						double bx =  x_vect_b;
						double by =  y_vect_b;
						double r_numerator = (d2_1_x-ax)*(bx-ax) + (d2_1_y-ay)*(by-ay);
						double r_denomenator = (bx-ax)*(bx-ax) + (by-ay)*(by-ay);
						double r = r_numerator / r_denomenator;
					    	double px = ax + r*(bx-ax);
					    	double py = ay + r*(by-ay);
						double xx = px;
						double yy = py;
					
						line.points[1].x = xx; line.points[1].y = yy;						
						//line.points[0].x = -4.7; line.points[0].y = -0.75; 
						//hector
						line.points[0].x = 1.8; line.points[0].y = -6.25;
						line.points[0].z = line.points[1].z = 0;
						
							}
				
						//Door1 frame 2
					if (count == 3){
						DistanceFromLine( d1_2_x, d1_2_y, ax, ay ,bx, by, distanceSegmentD1_12, 
						distanceLineD1_12);
						//double d1_2_x = -4.6; 
						//double d1_2_y =  4.0;
						// hector
						double d1_2_x = -4.1; 
						double d1_2_y = -7.4;
						double ax =  x_vect_a;
						double ay =  y_vect_a;
						double bx =  x_vect_b;
						double by =  y_vect_b;

						double r_numerator = (d1_2_x-ax)*(bx-ax) + (d1_2_y-ay)*(by-ay);
						double r_denomenator = (bx-ax)*(bx-ax) + (by-ay)*(by-ay);
						double r = r_numerator / r_denomenator;
					    	double px = ax + r*(bx-ax);
					    	double py = ay + r*(by-ay);
						double xx = px;
						double yy = py;
						
						line.points[1].x = xx; line.points[1].y = yy;
						//line.points[0].x = -4.6; line.points[0].y = 4.0; 
						//hector
						line.points[0].x = -4.1; line.points[0].y = -7.4; 
						line.points[0].z = line.points[1].z = 0;
						
							}
						
						//Door2 frame 2
					if (count == 4){
						DistanceFromLine( d2_2_x, d2_2_y, ax, ay ,bx, by, distanceSegmentD2_12, 
						distanceLineD2_12);
						//double d2_2_x = -4.7; 
						//double d2_2_y = -1.8;
						//hector
						double d2_2_x = 0.8; 
						double d2_2_y = -6.5;

						double ax =  x_vect_a;
						double ay =  y_vect_a;
						double bx =  x_vect_b;
						double by =  y_vect_b;

						double r_numerator = (d2_2_x-ax)*(bx-ax) + (d2_2_y-ay)*(by-ay);
						double r_denomenator = (bx-ax)*(bx-ax) + (by-ay)*(by-ay);
						double r = r_numerator / r_denomenator;
					    	double px = ax + r*(bx-ax);
					    	double py = ay + r*(by-ay);
						double xx = px;
						double yy = py;
						
						line.points[1].x = xx; line.points[1].y = yy;
						//line.points[0].x = -4.7; line.points[0].y = -1.8; 
						//hector
						line.points[0].x = 0.8; line.points[0].y = -6.5; 
						line.points[0].z = line.points[1].z = 0;
						
							}

						//Door1 frame1
					if (count == 5){
						DistanceFromLine( d1_1_x,  d1_1_y,  x_vect_b, y_vect_b, x_vect_c,  y_vect_c, 
						distanceSegmentD1_21, distanceLineD1_21);
						//double d1_1_x = -4.6; 
						//double d1_1_y =  5.3;
						//hector
						double d1_1_x = -5.1; 
						double d1_1_y = -7.5;

						double r_numerator = (d1_1_x-x_vect_b)*(x_vect_c-x_vect_b) + (d1_1_y-y_vect_b)*(y_vect_c-y_vect_b);
						double r_denomenator = (x_vect_c-x_vect_b)*(x_vect_c-x_vect_b) + (y_vect_c-y_vect_b)*(y_vect_c-y_vect_b);
						double r = r_numerator / r_denomenator;
					    	double px = x_vect_b + r*(x_vect_c-x_vect_b);
					    	double py = y_vect_b + r*(y_vect_c-y_vect_b);
						double xx = px;
						double yy = py;
						
						line.points[1].x = xx; line.points[1].y = yy;
						//line.points[0].x = -4.6; line.points[0].y = 5.3; 
						// hector
						line.points[0].x = -5.1; line.points[0].y =-7.5;
						line.points[0].z = line.points[1].z = 0;	
						
								}
						//Door2 frame1
					if (count == 6){
						DistanceFromLine( d2_1_x,  d2_1_y,  x_vect_b, y_vect_b, x_vect_c,  y_vect_c, 
						distanceSegmentD2_21, distanceLineD2_21);
						//double d2_1_x = -4.7; 
						//double d2_1_y = -0.75; 
						//hector
						double d2_1_x = 1.8; 
						double d2_1_y = -6.25; 
						double r_numerator = (d2_1_x-x_vect_b)*(x_vect_c-x_vect_b) + (d2_1_y-y_vect_b)*(y_vect_c-y_vect_b);
						double r_denomenator = (x_vect_c-x_vect_b)*(x_vect_c-x_vect_b) + (y_vect_c-y_vect_b)*(y_vect_c-y_vect_b);
						double r = r_numerator / r_denomenator;
					    	double px = x_vect_b + r*(x_vect_c-x_vect_b);
					    	double py = y_vect_b + r*(y_vect_c-y_vect_b);
						double xx = px;
						double yy = py;
						
						line.points[1].x = xx; line.points[1].y = yy;
						//line.points[0].x = -4.7; line.points[0].y = -0.75; 
						//hector
						line.points[0].x = 1.8; line.points[0].y = -6.25;
						line.points[0].z = line.points[1].z = 0;	
						
								}
						//Door1 frame2
					if (count == 7){
						DistanceFromLine( d1_2_x,  d1_2_y,  x_vect_b,  y_vect_b ,x_vect_c,  y_vect_c, 
						distanceSegmentD1_22, distanceLineD1_22);

						//double d1_2_x = -4.6; 
						//double d1_2_y =  4.0;
						//hector
						double d1_2_x = -4.1; 
						double d1_2_y = -7.4;
										
						double r_numerator = (d1_2_x-x_vect_b)*(x_vect_c-x_vect_b) + (d1_2_y-y_vect_b)*(y_vect_c-y_vect_b);
						double r_denomenator = (x_vect_c-x_vect_b)*(x_vect_c-x_vect_b) + (y_vect_c-y_vect_b)*(y_vect_c-y_vect_b);
						double r = r_numerator / r_denomenator;
					    	double px = x_vect_b + r*(x_vect_c-x_vect_b);
					    	double py = y_vect_b + r*(y_vect_c-y_vect_b);
						double xx = px;
						double yy = py;
						
						line.points[1].x = xx; line.points[1].y = yy;						
						//line.points[0].x = -4.6; line.points[0].y = 4.0; 
						//hector
						line.points[0].x = -4.1; line.points[0].y = -7.4;
						line.points[0].z = line.points[1].z = 0;
						
								}
						//Door2 frame2
					if (count == 8){
						DistanceFromLine( d2_2_x,  d2_2_y,  x_vect_b,  y_vect_b ,x_vect_c,  y_vect_c, 
						distanceSegmentD2_22, distanceLineD2_22);

						//double d2_2_x = -4.7; 
						//double d2_2_y = -1.8;
						//hector
						double d2_2_x = 0.8; 
						double d2_2_y = -6.5;
						
						double r_numerator = (d2_2_x-x_vect_b)*(x_vect_c-x_vect_b) + (d2_2_y-y_vect_b)*(y_vect_c-y_vect_b);
						double r_denomenator = (x_vect_c-x_vect_b)*(x_vect_c-x_vect_b) + (y_vect_c-y_vect_b)*(y_vect_c-y_vect_b);
						double r = r_numerator / r_denomenator;
					    	double px = x_vect_b + r*(x_vect_c-x_vect_b);
					    	double py = y_vect_b + r*(y_vect_c-y_vect_b);
						double xx = px;
						double yy = py;
						
						line.points[1].x = xx; line.points[1].y = yy;						
						//line.points[0].x = -4.7; line.points[0].y = -1.8; 
						//hector
						line.points[0].x = 0.8; line.points[0].y = -6.5; 
						line.points[0].z = line.points[1].z = 0;
						
								}
						//Door1 frame1
					if (count == 9){
						DistanceFromLine( d1_1_x,  d1_1_y,  x_vect_c,  y_vect_c ,x_vect_d,  y_vect_d, 
						distanceSegmentD1_31, distanceLineD1_31);

						//double d1_1_x = -4.6; 
					//double d1_1_y =  5.3;
						//hector
						double d1_1_x = -5.1; 
						double d1_1_y = -7.5;
						
						double r_numerator = (d1_1_x-x_vect_c)*(x_vect_d-x_vect_c) + (d1_1_y-y_vect_c)*(y_vect_d-y_vect_c);
						double r_denomenator = (x_vect_d-x_vect_c)*(x_vect_d-x_vect_c) + (y_vect_d-y_vect_c)*(y_vect_d-y_vect_c);
						double r = r_numerator / r_denomenator;
					    	double px = x_vect_c + r*(x_vect_d-x_vect_c);
					    	double py = y_vect_c + r*(y_vect_d-y_vect_c);
						double xx = px;
						double yy = py;
					
						line.points[1].x = xx; line.points[1].y = yy;						
						//line.points[0].x = -4.6; line.points[0].y = 5.3; 
						//hector
						line.points[0].x = -5.1; line.points[0].y = -7.5; 
						line.points[0].z = line.points[1].z = 0;
						
							}
						//Door2 frame1
					if (count == 10){
						DistanceFromLine( d2_1_x,  d2_1_y,  x_vect_c,  y_vect_c ,x_vect_d,  y_vect_d, 
						distanceSegmentD2_31, distanceLineD2_31);

						//double d2_1_x = -4.7; 
						//double d2_1_y = -0.75;
						//hector
						double d2_1_x = 1.8; 
						double d2_1_y = -6.25;

						double r_numerator = (d2_1_x-x_vect_c)*(x_vect_d-x_vect_c) + (d2_1_y-y_vect_c)*(y_vect_d-y_vect_c);
						double r_denomenator = (x_vect_d-x_vect_c)*(x_vect_d-x_vect_c) + (y_vect_d-y_vect_c)*(y_vect_d-y_vect_c);
						double r = r_numerator / r_denomenator;
					    	double px = x_vect_c + r*(x_vect_d-x_vect_c);
					    	double py = y_vect_c + r*(y_vect_d-y_vect_c);
						double xx = px;
						double yy = py;
					
						line.points[1].x = xx; line.points[1].y = yy;						
						//line.points[0].x = -4.7; line.points[0].y = -0.75; 
						//hector
						line.points[0].x = 1.8; line.points[0].y = -6.25; 
						line.points[0].z = line.points[1].z = 0;
						
							}
						//Door1 frame2
					if (count == 11){
						DistanceFromLine( d1_2_x,  d1_2_y,  x_vect_c,  y_vect_c ,x_vect_d,  y_vect_d, 
						distanceSegmentD1_32, distanceLineD1_32);

					 //double d1_2_x = -4.6; 
						//double d1_2_y =  4.0;
						//hector
						double d1_2_x = -4.1; 
						double d1_2_y = -7.4;						
						double r_numerator = (d1_2_x-x_vect_c)*(x_vect_d-x_vect_c) + (d1_2_y-y_vect_c)*(y_vect_d-y_vect_c);
						double r_denomenator = (x_vect_d-x_vect_c)*(x_vect_d-x_vect_c) + (y_vect_d-y_vect_c)*(y_vect_d-y_vect_c);
						double r = r_numerator / r_denomenator;
						double px = x_vect_c + r*(x_vect_d-x_vect_c);
						double py = y_vect_c + r*(y_vect_d-y_vect_c);
						double xx = px;
						double yy = py;
						line.points[1].x = xx; line.points[1].y = yy;						
						//line.points[0].x = -4.6; line.points[0].y =4.0;
						//hector
						line.points[0].x =-4.1; line.points[0].y = -7.4;
					 	line.points[0].z = line.points[1].z = 0;
							}
							
						//Door2 frame2
					if (count == 12){
						DistanceFromLine( d2_2_x,  d2_2_y,  x_vect_c,  y_vect_c ,x_vect_d,  y_vect_d, 
						distanceSegmentD2_32, distanceLineD2_32);

						//double d2_2_x = -4.7; 
					//double d2_2_y = -1.8;
						//hector
						double d2_2_x = 0.8; 
						double d2_2_y = -6.5;
												
						double r_numerator = (d2_2_x-x_vect_c)*(x_vect_d-x_vect_c) + (d2_2_y-y_vect_c)*(y_vect_d-y_vect_c);
						double r_denomenator = (x_vect_d-x_vect_c)*(x_vect_d-x_vect_c) + (y_vect_d-y_vect_c)*(y_vect_d-y_vect_c);
						double r = r_numerator / r_denomenator;
						double px = x_vect_c + r*(x_vect_d-x_vect_c);
						double py = y_vect_c + r*(y_vect_d-y_vect_c);
						double xx = px;
						double yy = py;
						line.points[1].x = xx; line.points[1].y = yy;						
						//line.points[0].x = -4.7; line.points[0].y =-1.8; 
						//hector
						line.points[0].x = 0.8; line.points[0].y =-6.5; 
						line.points[0].z = line.points[1].z = 0;
							}
						//Door1 frame1
					if (count == 13){
						DistanceFromLine( d1_1_x,  d1_1_y,  x_vect_d,  y_vect_d , x_vect_a,  y_vect_a,
						distanceSegmentD1_41, distanceLineD1_41);

						//double d1_1_x = -4.6; 
						//double d1_1_y =  5.3;
						//hector
						double d1_1_x =-5.1; 
						double d1_1_y = -7.5;
						double r_numerator = (d1_1_x-x_vect_d)*(x_vect_a-x_vect_d) + (d1_1_y-y_vect_d)*(y_vect_a-y_vect_d);
						double r_denomenator = (x_vect_a-x_vect_d)*(x_vect_a-x_vect_d) + (y_vect_a-y_vect_d)*(y_vect_a-y_vect_d);
						double r = r_numerator / r_denomenator;
						double px = x_vect_d + r*(x_vect_a-x_vect_d);
						double py = y_vect_d + r*(y_vect_a-y_vect_d);
						double xx = px;
						double yy = py;
						line.points[1].x = xx; line.points[1].y = yy;						
						//line.points[0].x = -4.6; line.points[0].y = 5.3; 
						//hector
						line.points[0].x = -5.1; line.points[0].y = -7.5;
						line.points[0].z = line.points[1].z = 0;
							}
						//Door2 frame1
					if (count == 14){
						DistanceFromLine( d2_1_x,  d2_1_y,  x_vect_d,  y_vect_d , x_vect_a,  y_vect_a,
						distanceSegmentD2_41, distanceLineD2_41);
						//double d2_1_x = -4.7; 
						//double d2_1_y = -0.75;
						//hector
						double d2_1_x = 1.8; 
						double d2_1_y = -6.25;
						double r_numerator = (d2_1_x-x_vect_d)*(x_vect_a-x_vect_d) + (d2_1_y-y_vect_d)*(y_vect_a-y_vect_d);
						double r_denomenator = (x_vect_a-x_vect_d)*(x_vect_a-x_vect_d) + (y_vect_a-y_vect_d)*(y_vect_a-y_vect_d);
						double r = r_numerator / r_denomenator;
						double px = x_vect_d + r*(x_vect_a-x_vect_d);
						double py = y_vect_d + r*(y_vect_a-y_vect_d);
						double xx = px;
						double yy = py;
						line.points[1].x = xx; line.points[1].y = yy;						
						//line.points[0].x = -4.7; line.points[0].y = -0.75; 
						//hector
						line.points[0].x = 1.8; line.points[0].y = -6.25;
						line.points[0].z = line.points[1].z = 0;
							}

						//Door1 frame2
					if (count == 15){
						DistanceFromLine( d1_2_x,  d1_2_y,  x_vect_d,  y_vect_d , x_vect_a,  y_vect_a, 
						distanceSegmentD1_42, distanceLineD1_42);
						//double d1_2_x = -4.6; 
						//double d1_2_y =  4.0;
						//hector
						double d1_2_x =-4.1; 
						double d1_2_y =-7.4;
						double r_numerator = (d1_2_x-x_vect_d)*(x_vect_a-x_vect_d) + (d1_2_y-y_vect_d)*(y_vect_a-y_vect_d);
						double r_denomenator = (x_vect_a-x_vect_d)*(x_vect_a-x_vect_d) + (y_vect_a-y_vect_d)*(y_vect_a-y_vect_d);
						double r = r_numerator / r_denomenator;
						double px = x_vect_d + r*(x_vect_a-x_vect_d);
						double py = y_vect_d + r*(y_vect_a-y_vect_d);
						double xx = px;
						double yy = py;
						line.points[1].x = xx; line.points[1].y = yy;						
						//line.points[0].x = -4.6; line.points[0].y =4.0;
						//hector
						line.points[0].x = -4.1; line.points[0].y = -7.4;
						line.points[0].z = line.points[1].z = 0;
							}
						//Door2 frame2
					if (count == 16){
						DistanceFromLine( d2_2_x,  d2_2_y,  x_vect_d,  y_vect_d , x_vect_a,  y_vect_a, 
						distanceSegmentD2_42, distanceLineD2_42);
						//double d2_2_x = -4.7; 
					 //double d2_2_y = -1.8;
						//hector
						double d2_2_x = 0.8; 
						double d2_2_y = -6.5;
						double r_numerator = (d2_2_x-x_vect_d)*(x_vect_a-x_vect_d) + (d2_2_y-y_vect_d)*(y_vect_a-y_vect_d);
						double r_denomenator = (x_vect_a-x_vect_d)*(x_vect_a-x_vect_d) + (y_vect_a-y_vect_d)*(y_vect_a-y_vect_d);
						double r = r_numerator / r_denomenator;
						double px = x_vect_d + r*(x_vect_a-x_vect_d);
						double py = y_vect_d + r*(y_vect_a-y_vect_d);
						double xx = px;
						double yy = py;
						//line.points[1].x = xx; line.points[1].y = yy;						
						//line.points[0].x = -4.7; line.points[0].y =-1.8; 
						//hector
						line.points[0].x = 0.8; line.points[0].y =-6.5;
						line.points[0].z = line.points[1].z = 0;
							}
							


								//if (mindistance < 1) {
				//ROS_INFO("Min distance to Door in m = %f", mindistance);
				//myfile << mindistance << "\n"; 
						    //}
				



						if (mindistance < total_min_distance) {
						total_min_distance=mindistance;
						   }
						ROS_INFO("Min distance to Door in m = %f", total_min_distance);
						myfile << total_min_distance << "\n"; 

			rate.sleep();
						 

			}

  return 0;
};

