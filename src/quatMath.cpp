#include "quatMath.h"

Eigen::Matrix3d quat2rot(const Eigen::VectorXd& q){
  Eigen::Matrix3d T = (pow(q(4),2)-q.head(3).transpose()*q.head(3))*Eigen::MatrixXd::Identity(3,3)
                      *2*q.head(3)*q.head(3).transpose()-2*q(4)*crossProductEquivalent(q.head(3));
  return T;
}
Eigen::VectorXd quatRot(const Eigen::VectorXd& q, const Eigen::VectorXd& dq){
  Eigen::VectorXd qOut = (Eigen::MatrixXd::Identity(4,4)*dq(4)+skew(dq))*q;
}
Eigen::MatrixXd skew(const Eigen::VectorXd& q){
  Eigen::MatrixXd mat(4,4);
  mat << 0    ,q(3) ,-q(2),q(1),
         -q(3),0    ,q(1) ,q(2),
         q(2) ,-q(1),0    ,q(3),
         -q(1),-q(2),-q(3),0   ;
  return mat;
}
Eigen::Matrix3d crossProductEquivalent(const Eigen::Vector3d& a){
  Eigen::Matrix3d ax;
  ax << 0,-a(2),a(1),
        a(2),0,-a(0),
        -a(1),a(0),0;
  return ax;
}