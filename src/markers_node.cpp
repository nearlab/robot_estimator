/* TODO: Publish markers for all bots to their own topic. 
         I'm sure this will just be something to do with launch files and smart coding.*/
#include <ros/ros.h>
#include <Eigen/Dense> 
#include <vicon_bridge/Markers.h>
#include <vicon_bridge/Marker.h>
#include "robot_controller/Markers.h"
#include <math.h> 
#include <string>

ros::Publisher pub;
ros::Subscriber subMarkers;
double[15] zMarkers;

ros::Time tsMarkers;


void markersCallback(const vicon_bridge::Markers msg){
  tsMarkers = msg.header.stamp;
  vicon_bridge::Markers markers = msg.markers;
  int s = markers.size();
  int zIter = 0;
  // while(operating){
  //   waitRate.sleep();
  // }
  // editing = true;
  for(int i=0;i<s;i++){
    if(markers[i].compare(robotName)){
      if(!markers[i].occluded){
        zMarkers[zIter++] = markers[i].x;
        zMarkers[zIter++] = markers[i].y;
        zMarkers[zIter++] = markers[i].z;
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
	std::string robotName;
	ros::NodeHandle nh;
	nh.getParam("RobotName", robotName);
  ros::init(argc,argv,"markers");
	subMarkers=nh.subscribe("vicon_bridge/Markers",1000,markersCallback);
  pub=nh.advertise<robot_controller::Markers>(strcat(robotName,"/markers"),1000);
  ros::Rate loop_rate(100);
  while(ros::ok()){
    robot_controller::Markers toPub;
    toPub.markers = zMarkers;
    toPub.tStamp = tsMarkers;
    pub.publish(toPub);
    
    ros::spinOnce();
    loop_rate.sleep();		
	}
}