/* TODO: Generate 3D LQR control matrix*/
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
Eigen::MatrixXd K;

ros::Publisher pub;
ros::Subscriber subState;
void stateCallback(const robot_controller::State msg){
  r << msg.r[0],msg.r[1],msg.r[2];
  q << msg.q[0],msg.q[1],msg.q[2],msg.q[3];
  v << msg.v[0],msg.v[1],msg.v[2];
}
int main(int argc, char** argv){
  K = Eigen::MatrixXd(3,6);
  K << 4.4814, -.2019, 0, 42.5742, .0015, 0,
     .2019, 4.4676, 0, .0015, 42.5092, 0,
     0, 0, 4.4675, 0, 0, 42.5090;
  double Torb;
  std::string robotNameStr;
  ros::NodeHandle nh;
  nh.getParam("RobotName", robotNameStr);
  nh.getParam("Torb", Torb);
  char robotName[robotNameStr.size() + 1];
  strcpy(robotName,robotNameStr.c_str());
  double n = 2*3.14159/Torb;

  ros::init(argc,argv,strcat(robotName,"/dynamics"));
  subState=nh.subscribe(strcat(robotName,"/state"),1000,stateCallback);
  pub=nh.advertise<robot_controller::Control>(strcat(robotName,"/control"),1000);
  ros::Rate loop_rate(100);
  while(ros::ok()){
    double radEst = r.norm();
    double radDes = 5;//meters in the space frame, I guess. Honestly, this is a bad idea, since S/C don't orbit that close. 
    double period = 86400.0/16.5/4;
    double velDes = 2*3.14159*radDes/period; //Note that we're orbiting a sc while orbiting the earth... while orbiting the sun while orbiting the galaxy etc etc
    
    Eigen::Vector3d rDes = radDes*(r/radEst);
    rDes(2) = 0;

    Eigen::Vector3d vDes;
    vDes << -velDes*r(1)/radEst,velDes*r(0)/radEst,0;

    Eigen::VectorXd rvErr(6);
    rvErr << rDes - r, vDes - v;

    f = K * rvErr;

    robot_controller::Control toPub;
    for(int i=0;i<3;i++){
      toPub.f[i] = f(i);
    }
    pub.publish(toPub);
    
    ros::spinOnce();
    loop_rate.sleep();		
	}
}
