#ifndef PARAMS_H
#define PARAMS_H

#include <Eigen/Dense>
#include <math.h>

class Params{
public:
  Params();
  Eigen::Matrix3d Qa,Qg,Qa0,Qg0,Qa2,Qmark;
  Eigen::MatrixXd RMarkers(15,15),Q(9,9), markerLocs(5,3);
};
#endif