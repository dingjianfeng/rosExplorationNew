#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include "stdint.h"
#include "functions.h"
#include "mtrand.h"


#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_listener.h>



// global variables
nav_msgs::OccupancyGrid mapData;




//Subscribers callback functions---------------------------------------
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
mapData=*msg;

std::cout<<"local map test by ding"<<std::endl;
}


int main(int argc, char **argv)
{

  
// generate the same numbers as in the original C test program
  ros::init(argc, argv, "map_test");
  ros::NodeHandle nh;

  std::cout<<"test by dingjianfeng"<<std::endl;
  
  

 
//---------------------------------------------------------------
ros::Subscriber sub= nh.subscribe("map", 100 ,mapCallBack);	
//ros::Subscriber rviz_sub= nh.subscribe("/clicked_point", 100 ,rvizCallBack);	

ros::spin();

return 0;

}





