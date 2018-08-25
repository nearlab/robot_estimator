/* TODO: Publish markers for all bots to their own topic. 
         I'm sure this will just be something to do with launch files and smart coding.*/
#include <ros/ros.h>
#include <Eigen/Dense> 
#include <vicon_bridge/Markers.h>
#include <vicon_bridge/Marker.h>
#include "robot_controller/Markers.h"
#include <boost/array.hpp>
#include <math.h> 
#include <string>
#include <cstring>
#include <vector>

ros::Publisher pub;
ros::Subscriber subMarkers;
boost::array<double, 15> zMarkers;
std::string robotName;
ros::Time tsMarkers;


void markersCallback(const vicon_bridge::Markers msg){
  tsMarkers = msg.header.stamp;
  std::vector<vicon_bridge::Marker> markers = msg.markers;
  int s = markers.size();
  int zIter = 0;
  // while(operating){
  //   waitRate.sleep();
  // }
  // editing = true;
  for(int i=0;i<s;i++){
    std::string markerName = markers[i].subject_name;
    if((markerName.compare(robotName))==0){
      if(!markers[i].occluded){
        zMarkers[zIter++] = markers[i].translation.x/1000;
        zMarkers[zIter++] = markers[i].translation.y/1000;
        zMarkers[zIter++] = markers[i].translation.z/1000;
      }else{
        zMarkers[zIter++] = std::numeric_limits<double>::quiet_NaN();
        zMarkers[zIter++] = std::numeric_limits<double>::quiet_NaN();
        zMarkers[zIter++] = std::numeric_limits<double>::quiet_NaN();
      }
    } 
  }
  //editing = false;
}
int main(int argc, char** argv){
  ros::init(argc,argv,"markers");
  ros::NodeHandle nh;
  nh.getParam("RobotName", robotName);
  tsMarkers = ros::Time(0);

  subMarkers=nh.subscribe("vicon/markers",1000,markersCallback);
  pub=nh.advertise<robot_controller::Markers>(robotName+std::string("/markers"),1000);
  ros::Rate loop_rate(100);
  ROS_INFO("Markers Node Initialized");
  while(ros::ok()){
    if(tsMarkers.toSec() == 0){
      ros::spinOnce();
      loop_rate.sleep();		
      continue;
    }
    robot_controller::Markers toPub;
    toPub.markers = zMarkers;
    toPub.tStamp = tsMarkers.toSec();
    pub.publish(toPub);
    
    ros::spinOnce();
    loop_rate.sleep();		
	}
}
