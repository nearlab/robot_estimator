#include <ros/ros.h>
#include <Eigen/Dense> 
#include <geometry_msgs/Vector3.h>
#include "cans_msgs/State.h"
#include "cans_msgs/Control.h"
#include <math.h> 
#include <string.h>

Eigen::Vector3d r,v,a,f;
Eigen::Vector4d q;//Unused as of yet
double n = 2*pi/T; //TODO: Get Torb from a launch file somehow. Scoping is hard. Globals, perhaps.
ros::Publisher pub;
ros::Subscriber subState, subControl;
void stateCallback(const cans_msgs::State msg){
  r << msg.r;
  q << msg.q;
  v << msg.v;
}
void controlCallback(const cans_msgs::Control msg){
  f << msg.f;
}
int main(int argc, char** argv){
	node.getParam("RobotName", robotName);
  ros::init(argc,argv,strcat(robotName,"/dynamics"));
	ros::NodeHandle nh;
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
