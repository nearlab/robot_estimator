/* TODO: Publish markers for all bots to their own topic. 
         I'm sure this will just be something to do with launch files and smart coding.*/
#include <ros/ros.h>
#include <Eigen/Dense> 
#include <vicon_bridge/Markers.h>
#include <vicon_bridge/Marker.h>
#include "robot_controller/Markers.h"
#include <math.h> 
#include <string>
#include <cstring>
#include <vector>

ros::Publisher pub;
ros::Subscriber subMarkers;
std::vector<double> zMarkers;
std::string robotNameStr;
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
    if(markers[i].subject_name.compare(robotName)){
      if(!markers[i].occluded){
        zMarkers[zIter++] = markers[i].translation.x;
        zMarkers[zIter++] = markers[i].translation.y;
        zMarkers[zIter++] = markers[i].translation.z;
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
  zMarkers = std::vector<double>(15);
  ros::NodeHandle nh;
  nh.getParam("RobotName", robotNameStr);
  char robotName[robotNameStr.size() + 1];
  strcpy(robotName,robotNameStr.c_str());
  ros::init(argc,argv,"markers");
	subMarkers=nh.subscribe("vicon_bridge/Markers",1000,markersCallback);
  pub=nh.advertise<robot_controller::Markers>(strcat(robotName,"/markers"),1000);
  ros::Rate loop_rate(100);
  while(ros::ok()){
    robot_controller::Markers toPub;
    toPub.markers = zMarkers;
    toPub.tStamp = tsMarkers.toSec();
    pub.publish(toPub);
    
    ros::spinOnce();
    loop_rate.sleep();		
	}
}
