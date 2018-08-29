/* TODO: Also publish the state scaled to orbit. */
#include <ros/ros.h>
#include <time.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense> 

#include "robot_controller/State.h"
#include "robot_controller/MarkersParsed.h"
#include "robot_controller/Imu.h"
#include "robot_controller/StateCov.h"
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

void markersCallback(const robot_controller::MarkersParsed msg){
  tsMarkers = ros::Time(msg.tStamp);
  for(int i=0;i<15;i++){
    zMarkers(i) = msg.markers[i];
  }
}   

void imuCallback(const robot_controller::Imu msg){
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

  subImu = nh.subscribe(robotName+std::string("/imu"),1000,imuCallback);
  subMarkers = nh.subscribe(robotName+std::string("/markers"),1000,markersCallback);
  pubState = nh.advertise<robot_controller::State>(robotName+std::string("/state"),1000);
  pubStateCov = nh.advertise<robot_controller::StateCov>(robotName+std::string("/state_cov"),1000);
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
      ROS_INFO_STREAM("zImu:"<<zImu<<"\ndtImu:"<<dtImu);
      double dtMarkers = ((ros::Duration)(tsMarkers - tsMarkersOld)).toSec();
      estimator.predict(zImu,dtImu);
      // operating = true;
      // while(editing){
      //   waitRate.sleep();
      // }
      estimator.correct(zMarkers,dtMarkers);
      // operating = false;
    }
    tsImuOld = ros::Time().fromNSec(tsImu.toNSec());
    tsMarkersOld = ros::Time().fromNSec(tsMarkers.toNSec());
    
    robot_controller::State stateMsg;
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

    robot_controller::StateCov stateCovMsg;
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
