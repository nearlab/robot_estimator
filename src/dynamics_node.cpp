#include <ros/ros.h>
#include <Eigen/Dense> 
#include <geometry_msgs/Vector3.h>
#include "robot_estimator/Control.h"
#include "robot_estimator/State.h"
#include "robot_estimator/SpacecraftState.h"
#include <math.h> 
#include <string>
#include <cstring>

Eigen::Vector3d r,v,a,f;
Eigen::Vector4d q;//Unused as of yet
ros::Time tsControl,tsState;
double pi = 3.14159265358979;
ros::Publisher pub;
ros::Subscriber subState, subControl;
void stateCallback(const robot_estimator::State msg){
  r << msg.r[0],msg.r[1],msg.r[2];
  q << msg.q[0],msg.q[1],msg.q[2],msg.q[3];
  v << msg.v[0],msg.v[1],msg.v[2];
  tsState = ros::Time(msg.tStamp);
}
void controlCallback(const robot_estimator::Control msg){
  f << msg.f[0],msg.f[1],msg.f[2];
  tsControl = ros::Time(msg.tStamp);
}
int main(int argc, char** argv){
  ros::init(argc,argv,"dynamics");
  std::string robotName;
  double Torb,mass,tau,nu;
  ros::NodeHandle nh;
  nh.getParam("RobotName", robotName);
  nh.getParam("Torb", Torb);
  nh.getParam("sc_mass", mass);
  nh.getParam("tau",tau);
  nh.getParam("nu",nu);
  double n = 2*pi/Torb; //Mean motion 
  tsControl = ros::Time(0);
  tsState = ros::Time(0);

	
  subState=nh.subscribe(robotName+std::string("/state"),1000,stateCallback);
  subControl=nh.subscribe(robotName+std::string("/control"),1000,controlCallback);
  pub=nh.advertise<robot_estimator::SpacecraftState>(robotName+std::string("/sc_state"),2);
  
  ros::Rate loop_rate(100);
  ROS_INFO("Dynamics Node Initialized");
  while(ros::ok()){
    if(tsControl.toSec() == 0 || tsState.toSec() == 0){
      ros::spinOnce();
      loop_rate.sleep();		
      continue;
    }
    a(0) = 3*n*n*r(0)+2*v(1)*n+f(0)/mass;
    a(1) = -2*v(1)*n+f(1)/mass;
    a(2) = -n*n*r(2)+f(2)/mass;
    robot_estimator::SpacecraftState out;
    for(int i=0;i<3;i++){
      out.a[i] = a(i)*tau*tau/nu;
      out.r[i] = r(i)/nu;
      out.v[i] = v(i)*tau/nu;
      out.q[i] = q(i);
    }
    out.q[3] = q(3);
    out.tStamp = (ros::Time::now()).toSec();

    pub.publish(out);
    
    ros::spinOnce();
    loop_rate.sleep();		
	}
}
