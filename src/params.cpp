/* TODO: MAKE THESE COME FROM A LAUNCH OR CONFIG FILE. MAYBE READ IN DATA FROM VICON FOR MARKERLOCS */
#include "params.h"

Params::Params(){
  double sa,sg,sigmaa0,sigmag0,sigmaa,taua,smarker,imuDelt;
  RMarkers = Eigen::MatrixXd(15,15);
  Q = Eigen::MatrixXd(9,9);
  markerLocs = Eigen::MatrixXd(5,3);
  double pi = 3.14159265358979;
  imuDelt = .01;
  //Accelerometer white noise
  sa = pow((.08/sqrt(imuDelt)),2);
  this->Qa.setIdentity();
  this->Qa *= pow((9.8/1000),2)*sa;

  //Gyro white noise
  sg = pow((.01/sqrt(imuDelt)),2);
  this->Qg.setIdentity();
  this->Qg *= pow(pi/180,2)*sg;

  //Accelerometer startup bias
  sigmaa0 = .05;
  this->Qa0.setIdentity();
  this->Qa0 *= pow(sigmaa0,2);

  //Gyro startup bias
  sigmag0 = .2;
  this->Qg0.setIdentity();
  this->Qg0 *= pi/180*pow(sigmag0,2);

  //Accelerometer bias white noise
  sigmaa = 15*9.8/1000;
  taua = 10;
  this->Qa2.setIdentity();
  this->Qa2 *= pow(sigmaa,2)*(1-exp(-2*imuDelt/taua));

  //Set up dynamics covariance matrix
  this->Q.setIdentity();
  this->Q.topLeftCorner(3,3) = Qa;
  this->Q.block(3,3,3,3) = Qg;
  this->Q.bottomRightCorner(3,3) = Qa2;

  //Set up measurement covariance matrix
  this->Qmark.setIdentity();
  this->Qmark *= pow(.0014,2);
  this->RMarkers.setIdentity();
  for(int i=0;i<5;i++){
    this->RMarkers.block(i,i,3,3) = this->Qmark;
  }

  this->markerLocs << 124.853,-7.45219,-56.7298,
                      208.978,-136.781,-60.2774,
                      -19.8742,-14.15,61.4902,
                      -75.9264,43.3454,106.429,
                      -238.031,115.037,-50.9121;
  this->markerLocs *= .001;
}
