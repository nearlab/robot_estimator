/* TODO: Publish markers for all bots to their own topic. 
         I'm sure this will just be something to do with launch files and smart coding.*/
#include <ros/ros.h>
#include <Eigen/Dense> 
#include <Eigen/Cholesky>
#include <geometry_msgs/TransformStamped.h>
#include "robot_controller/Imu.h"
#include "quatMath.h"
#include "params.h"
#include <boost/array.hpp>
#include <math.h> 
#include <string>
#include <cstring>
#include <vector>
#include <random>

ros::Publisher pub;
ros::Subscriber subVicon;
Eigen::Vector3d ba,bg,va,vg,acc,gyr;
Eigen::MatrixXd stateHist;
int histCount;
std::string robotName;
ros::Time tsVicon;
Params params;
std::normal_distribution<double> randDist;
std::default_random_engine randGen;

Eigen::MatrixXd qa0Decomp, qg0Decomp, qaDecomp,qgDecomp,qa2Decomp;

Eigen::VectorXd randn(int n){
  Eigen::VectorXd randVec(n);
  for(int i=0;i<n;i++){
    randVec(i) = randDist(randGen);
  }
  return randVec;
}

void setupChol(){
  params = Params();
  qa0Decomp = params.Qa0.llt().matrixL();
  qaDecomp = params.Qa.llt().matrixL();
  qa2Decomp = params.Qa2.llt().matrixL();
  qg0Decomp = params.Qg0.llt().matrixL();
  qgDecomp = params.Qg.llt().matrixL();
  ba = qa0Decomp * randn(3);
  bg = qg0Decomp * randn(3);
}

void viconCallback(const geometry_msgs::TransformStamped msg){
  
  if(histCount < 3){
    stateHist(0,histCount) = msg.transform.translation.x;
    stateHist(1,histCount) = msg.transform.translation.y;
    stateHist(2,histCount) = msg.transform.translation.z;
    stateHist(3,histCount) = msg.transform.rotation.x;
    stateHist(4,histCount) = msg.transform.rotation.y;
    stateHist(5,histCount) = msg.transform.rotation.z;
    stateHist(6,histCount) = msg.transform.rotation.w;
    stateHist(7,histCount) = msg.header.stamp.toSec();
    histCount++;
  }
  else{
    tsVicon = msg.header.stamp;
    stateHist.col(0) = stateHist.col(1);
    stateHist.col(1) = stateHist.col(2);
    stateHist(0,2) = msg.transform.translation.x;
    stateHist(1,2) = msg.transform.translation.y;
    stateHist(2,2) = msg.transform.translation.z;
    stateHist(3,2) = msg.transform.rotation.x;
    stateHist(4,2) = msg.transform.rotation.y;
    stateHist(5,2) = msg.transform.rotation.z;
    stateHist(6,2) = msg.transform.rotation.w;
    stateHist(7,2) = msg.header.stamp.toSec();

    Eigen::Vector4d q1 = stateHist.block(3,1,4,1);
    Eigen::Vector4d q2 = stateHist.block(3,2,4,1);
    Eigen::Vector4d dq = quatRot(q2,inverse(q1));
    gyr = dq.segment(0,3);

    double t1 = stateHist(7,1) - stateHist(7,0);
    double t2 = stateHist(7,2) - stateHist(7,0);
    for(int i=0;i<3;i++){
      acc(i) = (stateHist(i,2)-(stateHist(i,1)-stateHist(i,0))*t2/t1-stateHist(i,0))/t2/t2;
    }

    va = qaDecomp * randn(3);
    vg = qaDecomp * randn(3);
    Eigen::Vector3d va2 = qa2Decomp * randn(3);
    ba += va2;
  }
}

int main(int argc, char** argv){
  ros::init(argc,argv,"imu_sim");
  ros::NodeHandle nh;
  nh.getParam("RobotName", robotName);
  tsVicon = ros::Time(0);
  histCount = 0;
  stateHist = Eigen::MatrixXd::Zero(8,3);
  randDist = std::normal_distribution<double>(0.0,1.0);
  setupChol();

  subVicon=nh.subscribe(std::string("vicon/")+robotName+std::string("/")+robotName,1000,viconCallback);
  pub=nh.advertise<robot_controller::Imu>(robotName+std::string("/imu"),1000);
  ros::Rate loop_rate(100);
  ROS_INFO("IMU Sim Node Initialized");
  while(ros::ok()){
    if(tsVicon.toSec() == 0){
      ros::spinOnce();
      loop_rate.sleep();		
      continue;
    }
    robot_controller::Imu toPub;
    for(int i=0;i<3;i++){
      toPub.accTruth[i] = acc(i);
      toPub.gyrTruth[i] = gyr(i);
      toPub.acc[i] = acc(i) + ba(i) + va(i);
      toPub.gyr[i] = gyr(i) + bg(i) + vg(i);
    }
    toPub.tStamp = tsVicon.toSec();
    pub.publish(toPub);
    
    ros::spinOnce();
    loop_rate.sleep();		
  }
}
