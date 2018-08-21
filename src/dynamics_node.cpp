#include <ros/ros.h>
#include <Eigen/Dense> 
#include <geometry_msgs/Vector3.h>
#include "robot_controller/Control.h"
#include "robot_controller/State.h"
#include <math.h> 
#include <string>
#include <cstring>

Eigen::Vector3d r,v,a,f;
Eigen::Vector4d q;//Unused as of yet
double pi = 3.14159265358979;
ros::Publisher pub;
ros::Subscriber subState, subControl;
void stateCallback(const robot_controller::State msg){
  r << msg.r[0],msg.r[1],msg.r[2];
  q << msg.q[0],msg.q[1],msg.q[2],msg.q[3];
  v << msg.v[0],msg.v[1],msg.v[2];
}
void controlCallback(const robot_controller::Control msg){
  f << msg.f[0],msg.f[1],msg.f[2];
}
int main(int argc, char** argv){
  std::string robotNameStr;
  double Torb,mass;
  ros::NodeHandle nh;
  nh.getParam("RobotName", robotNameStr);
  nh.getParam("Torb", Torb);
  nh.getParam("mass", mass);
  double n = 2*pi/Torb; //Mean motion  
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
    geometry_msgs::Vector3 output;
    output.x = a(0);
    output.y = a(1);
    output.z = a(2);
    pub.publish(output);
    
    ros::spinOnce();
    loop_rate.sleep();		
	}
}
