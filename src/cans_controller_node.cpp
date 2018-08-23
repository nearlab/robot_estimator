/* TODO: Generate 3D LQR control matrix*/
#include <ros/ros.h>
#include <Eigen/Dense> 
#include <robot_controller/SpacecraftState.h>
#include <math.h> 
#include <string>
#include <cstring>
#include <iostream>
#include <fstream>

Eigen::Vector3d r,v,a,f;
Eigen::Vector4d q;//Unused as of yet
Eigen::MatrixXd K;

std::ofstream pwmWriter;

ros::Publisher pub;
ros::Subscriber subState;
void stateCallback(const robot_controller::SpacecraftState msg){
  r << msg.r[0],msg.r[1],msg.r[2];
  q << msg.q[0],msg.q[1],msg.q[2],msg.q[3];
  v << msg.v[0],msg.v[1],msg.v[2];
  a << msg.a[0],msg.a[1],msg.a[2];
}
void setupPWMs(){
  
  pwmWriter.open("/sys/class/pwm/pwmchip1/export",std::ofstream::out);
  pwmWriter << 0 <<std::endl;
  pwmWriter.close();
  pwmWriter.open("/sys/class/pwm/pwmchip1/pmw-1:0/period",std::ofstream::out);
  pwmWriter << 33333 <<std::endl;
  pwmWriter.close();
  pwmWriter.open("/sys/class/pwm/pwmchip1/pmw-1:0/enable",std::ofstream::out);
  pwmWriter << 1 <<std::endl;
  pwmWriter.close();


  pwmWriter.open("/sys/class/pwm/pwmchip4/export",std::ofstream::out);
  pwmWriter << 0 <<std::endl;
  pwmWriter.close();
  pwmWriter.open("/sys/class/pwm/pwmchip4/pmw-4:0/period",std::ofstream::out);
  pwmWriter << 33333 <<std::endl;
  pwmWriter.close();
  pwmWriter.open("/sys/class/pwm/pwmchip4/pmw-4:0/enable",std::ofstream::out);
  pwmWriter << 1 <<std::endl;
  pwmWriter.close();


  pwmWriter.open("/sys/class/pwm/pwmchip6/export",std::ofstream::out);
  pwmWriter << 0 <<std::endl;
  pwmWriter.close();
  pwmWriter.open("/sys/class/pwm/pwmchip6/pmw-6:0/period",std::ofstream::out);
  pwmWriter << 33333 <<std::endl;
  pwmWriter.close();
  pwmWriter.open("/sys/class/pwm/pwmchip6/pmw-6:0/enable",std::ofstream::out);
  pwmWriter << 1 <<std::endl;
  pwmWriter.close();
}
int main(int argc, char** argv){
  setupPWMs();
  ros::init(argc,argv,"cans_controller");
  std::string robotNameStr;
  ros::NodeHandle nh;
  double gamma,alpha,mass,radius,wheelDist,pi;
  pi = 3.14159265358979;
  nh.getParam("gamma",gamma);
  nh.getParam("alpha",alpha);
  nh.getParam("mass",mass);
  nh.getParam("radius",radius);
  nh.getParam("wheelDist",wheelDist);
  nh.getParam("RobotName", robotNameStr);
  char robotName[robotNameStr.size() + 1];
  strcpy(robotName,robotNameStr.c_str());
  double n = 2*pi/Torb;
  Eigen::Matrix3D D;
  D << 0, -sqrt(3)/2, sqrt(3)/2,
       1, -1/2      , -1/2     , 
       0, 0         , 0        ;

  subState=nh.subscribe(strcat(robotName,"/sc_state"),2,stateCallback);
  ros::Rate loop_rate(10);
  while(ros::ok()){
    Eigen::Matrix3d A1,A2,R; //From A1 * xdd + A2 * xd = u
    double theta = 2*atan2(q(3),q(2));
    double d = pi/3;
    R << -sin(theta),-sin(d-theta),sin(d+theta),
         cos(theta),-cos(d-theta),-cos(d+theta),
         1         , 1           , 1;
    A1 << mass/alpha, 0, 0,
          0, mass/alpha, 0,
          0, 0, .5*mass*radius*radius/alpha/wheelDist;
    A2 << 1.5*gamma/alpha,0,0,
          0, 1.5*gamma/alpha,0,
          0, 0, 3*gamma*wheelDist/alpha;
    thetaStar = atan2(r(2),r(1));
    if(thetaStar<-pi/2 && theta>pi/2){
      thetaStar = thetaStar + pi/2;
    }else if(thetaStar>pi/2 && theta<-pi/2){
      theta = theta + pi/2;
    }
    double correctionAngle = thetaStar - theta;
    Eigen::Vector3d u,x;
    x << r(1),r(2),correctionAngle;
    u = R.inverse()*(A1*a + A2*v);
    u = u/3.33*33333;
    //Write duty cycle to pwm files
    pwmWriter.open("/sys/class/pwm/pwmchip1/pmw-1:0/duty_cycle",std::ofstream::out);
    pwmWriter << u(0) <<std::endl;
    pwmWriter.close();
    pwmWriter.open("/sys/class/pwm/pwmchip4/pmw-4:0/duty_cycle",std::ofstream::out);
    pwmWriter << u(1) <<std::endl;
    pwmWriter.close();
    pwmWriter.open("/sys/class/pwm/pwmchip6/pmw-6:0/duty_cycle",std::ofstream::out);//std::ofstream::append, for logging.
    pwmWriter << u(2) <<std::endl;
    pwmWriter.close();

    ros::spinOnce();
    loop_rate.sleep();		
	}
}
