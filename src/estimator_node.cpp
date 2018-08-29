/* TODO: Also publish the state scaled to orbit. */
#include <ros/ros.h>
#include <time.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense> 

#include "robot_estimator/State.h"
#include "robot_estimator/MarkersParsed.h"
#include "robot_estimator/Imu.h"
#include "robot_estimator/StateCov.h"
#include "estimator.h"

#include <string>
#include <cstring>
#include <array>

Eigen::VectorXd zMarkers(15), zImu(6), state(12);
Eigen::MatrixXd P(12,12);
Estimator estimator;

// bool editing,operating;
// ros::Rate waitRate(10000);

ros::Publisher pubState, pubStateCov;
ros::Subscriber subMarkers, subImu;

ros::Time tsMarkers, tsImu;

void markersCallback(const robot_estimator::MarkersParsed msg){
  tsMarkers = ros::Time(msg.tStamp);
  for(int i=0;i<15;i++){
    zMarkers(i) = msg.markers[i];
  }
}   

void imuCallback(const robot_estimator::Imu msg){
    zImu << msg.accTruth[0],msg.accTruth[1],msg.accTruth[2],msg.gyrTruth[0],msg.gyrTruth[1],msg.gyrTruth[2];
    tsImu = ros::Time(msg.tStamp);
}

int main(int argc, char** argv){
  ros::init(argc,argv,"estimator");
  std::string robotName;
  ros::NodeHandle nh;
  nh.getParam("RobotName", robotName);
  tsImu = ros::Time(0);
  tsMarkers = ros::Time(0);

  subImu = nh.subscribe(robotName+std::string("/imu"),2,imuCallback);
  subMarkers = nh.subscribe(robotName+std::string("/markers"),2,markersCallback);
  pubState = nh.advertise<robot_estimator::State>(robotName+std::string("/state"),1000);
  pubStateCov = nh.advertise<robot_estimator::StateCov>(robotName+std::string("/state_cov"),1000);
  ros::Rate loop_rate(100);
  ros::Time tsImuOld,tsMarkersOld;
  bool first = true;
  ROS_INFO("Estimator Node Initialized");

  while(ros::ok()){
    if(first){
      //Do things
      if(tsImu.toSec() == 0 || tsMarkers.toSec() == 0){
        ros::spinOnce();
        loop_rate.sleep();
        continue;
      }
      estimator.estimateStateFromMarkers(zMarkers);
      ROS_INFO("Estimator Initialized");
      first=false;
    }else{
      double dtImu = ((ros::Duration)(tsImu - tsImuOld)).toSec();
      ROS_INFO_STREAM("zImu:"<<zImu<<"\ntsImu:"<<tsImu.toSec()<<"\ntsImuOld"<<tsImuOld.toSec()<<"\ndtImu:"<<dtImu);
      double dtMarkers = ((ros::Duration)(tsMarkers - tsMarkersOld)).toSec();
      estimator.predict(zImu,dtImu);
      ROS_INFO_STREAM("State:"<<estimator.getState());
      if(dtMarkers > 0){
        estimator.correct(zMarkers,dtMarkers);
        ROS_INFO_STREAM("State:"<<estimator.getState());
      }
    }
    tsImuOld = ros::Time().fromNSec(tsImu.toNSec());
    tsMarkersOld = ros::Time().fromNSec(tsMarkers.toNSec());
    
    robot_estimator::State stateMsg;
    state = estimator.getState();
    for(int i=0;i<3;i++){//There are better ways
      stateMsg.r[i] = state(i);
      stateMsg.q[i] = state(i+3);
      stateMsg.v[i] = state(i+7);
      stateMsg.ba[i] = state(i+10);
    }
    stateMsg.q[3] = state(6);
    ros::Time tsNow = ros::Time::now();
    stateMsg.tStamp = tsNow.toSec();
    pubState.publish(stateMsg);

    robot_estimator::StateCov stateCovMsg;
    P = estimator.getCovariance();
    for(int i=0;i<P.rows();i++){
      stateCovMsg.P[i] = P(i,i);
    }
    stateCovMsg.tStamp = tsNow.toSec();
    pubStateCov.publish(stateCovMsg);

    ros::spinOnce();
    loop_rate.sleep();
  }
}
