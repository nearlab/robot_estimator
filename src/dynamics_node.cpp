#include <ros/ros.h>
#include <Eigen/Dense> 
#include <geometry_msgs/Vector3.h>
#include "robot_controller/State.h"
#include "robot_controller/Control.h"
#include <math.h> 
#include <string>
#include <cstring>

Eigen::Vector3d r,v,a,f;
Eigen::Vector4d q;//Unused as of yet
double n = 2*pi/T; //TODO: Get Torb from a launch file somehow. Scoping is hard. Globals, perhaps.
ros::Publisher pub;
ros::Subscriber subState, subControl;
void stateCallback(const robot_controller::State msg){
  r << msg.r;
  q << msg.q;
  v << msg.v;
}
void controlCallback(const robot_controller::Control msg){
  f << msg.f;
}
int main(int argc, char** argv){
  std::string robotNameStr;
  ros::NodeHandle nh;
  nh.getParam("RobotName", robotNameStr);
  char robotName[robotNameStr.size() + 1];
  strcpy(robotName,robotNameStr.c_str());
  ros::init(argc,argv,strcat(robotName,"/dynamics"));
	
	subState=nh.subscribe(strcat(robotName,"/state"),1000,stateCallback);
  subControl=nh.subscribe(strcat(robotName,"/control"),1000,controlCallback);
  pub=nh.advertise<geometry_msgs::Vector3>(strcat(robotName,"/sc_accel"),1000);
  
  ros::Rate loop_rate(100);
  while(ros::ok()){
    a(0) = 3*n*n*r(0)+2*v(1)*n+f(0)/mass;
    a(1) = -2*v(1)*n+f(1)/mass;
    a(2) = -n*n*r(2)+f(2)/mass;
    geometry_msgs::Vector3 output(a(0),a(1),a(2));
    pub.publish(output);
    
    ros::spinOnce();
    loop_rate.sleep();		
	}
}
