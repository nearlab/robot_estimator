#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <boost/array.hpp>
#include <math.h>
#include <cmath>
#include "params.h"
#include "simulator.h"
#include "quatMath.h"
//#include <ros/console.h>
#include <ros/ros.h>

class Estimator{
public:
  Estimator();
  void predict(const Eigen::VectorXd& zImu, const double& dtImu);
  void correct(const Eigen::VectorXd& zMarkers, const double& dtMarkers);
  Eigen::MatrixXd parseMeasMarkers(const Eigen::VectorXd& zMarkers);
  void estimateStateFromMarkers(const Eigen::VectorXd& zMarkers);
  Eigen::VectorXd getState();
  Eigen::MatrixXd getCovariance();
  Params getParams();
  bool initialized();

private:
  Eigen::VectorXd state;  
  Eigen::MatrixXd P;
  bool isInitialized;
  Params params;
};
#endif
