/* TODO: Also publish the state scaled to orbit. */
#include <ros/ros.h>
#include <time.h>
#include <geometry_msgs/Point.h>
#include <vicon_bridge/Markers.h>
#include <vicon_bridge/Marker.h>
#include <Eigen/Dense> 

#include "cans_msgs/State.h"
#include "cans_msgs/Imu.h"
#include "estimator.h"
#include "estimateStateFromMarkers.h"

#include <string.h>
#include <array>

Eigen::VectorXd zMarkers(15), zImu(7), state(12);
Eigen::MatrixXd P(12,12);
Estimator estimator();

string robotName;
// bool editing,operating;
// ros::Rate waitRate(10000);

ros::Publisher pub;
ros::Subscriber subMarkers, subImu;

ros::Time tsMarkers, tsImu;

void markersCallback(const cans_msgs::Markers msg){
  tsMarkers = msg.tStamp;
  for(int i=0;i<15;i++){
    zMarkers(i) = msg.markers[i];
  }
}   

void imuCallback(const cans_msgs::Imu msg){
    zImu << msg.acc << msg.gyr;
}

int main(int argc, char** argv){
  robotName = argv[0];
	ros::init(argc,argv,strcat(robotName,"/estimator"));
	ros::NodeHandle nh;
	subImu = nh.subscribe(strcat(robotName,"/imu"),1000,stateCallback);
  subMarkers = nh.subscribe(strcat(robotName,"/markers"),1000,markersCallback);
  pub = nh.advertise<cans_msgs::State>(strcat(robotName,"/state"),1000);
  ros::Rate loop_rate(100);
  ros::Time tsImuOld,tsMarkersOld;
  bool first = true;
  while(ros::ok()){
    if(first){
      //Do things
      estimator.estimateStateFromMarkers(state,zMarkers,params);
    }else{
      double dtImu = ((ros::Duration)(tsImu - tsImuOld)).toSecs();
      double dtMarkers = ((ros::Duration)(tsMarkers - tsMarkersOld)).toSecs();
      estimator.predict(zImu,dtImu);
      // operating = true;
      // while(editing){
      //   waitRate.sleep();
      // }
      estimator.correct(zMarkers,dtMarkers);
      // operating = false;
    }
    tsImuOld = ros::Time.fromNSec(tsImu.toNSec());
    tsMarkersOld = ros::Time.fromNSec(tsMarkers.toNSec());
    cans_msgs::State toPub;
    state = estimator.getState();
    for(int i=0;i<3;i++){//There are better ways
      toPub.r(i) = state(i);
      toPub.q(i) = state(i+3);
      toPub.v(i) = state(i+7);
      toPub.ba(i) = state(i+10);
    }
    toPub.q(3) = state(6);
    pub.publish(toPub);
    ros::spinOnce();
    loop_rate.sleep();
	}
}
