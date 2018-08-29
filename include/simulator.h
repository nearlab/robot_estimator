#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <Eigen/Dense>
#include "quatMath.h"
#include <vector>
#include "params.h"
#include <ros/ros.h>
Eigen::VectorXd markerSimulator(const Eigen::VectorXd& state, const Params& params, const std::vector<int> occluded = {0,0,0,0,0});
Eigen::VectorXd imuSimulator(const Eigen::VectorXd& state, const double& dtImu, const Params& params);

#endif
