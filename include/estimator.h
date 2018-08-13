#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <Eigen/Dense>
#include <math.h>
#include "params.h"
#include "simulator.h"
#include "quatMath.h"

class Estimator{
public:
  Estimator();
  void predict(const Eigen::VectorXd& zImu, const double& dtImu);
  void correct(const Eigen::VectorXd& zMarkers, const double& dtMarkers);
  Eigen::MatrixXd parseMeasMarkers(const Eigen::VectorXd& zMarkers);
  void estimateStateFromMarkers(const Eigen::VectorXd& zMarkers);
  Eigen::VectorXd getState();
  Eigen::MatrixXd getCovariance()
  bool initialized();

private:
  Eigen::VectorXd state(13);  
  Eigen::MatrixXd P(12,12);
  bool isInitialized;
  Params params;
};
#endif