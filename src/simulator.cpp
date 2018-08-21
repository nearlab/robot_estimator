#include "simulator.h"

Eigen::VectorXd markerSimulator(const Eigen::VectorXd& state, const Params& params, const std::vector<int> occluded){
  int nOccl = 0;
  int n = params.markerLocs.col(0).size();
  Eigen::Matrix3d T = quat2rot(state.block(3,4));
  Eigen::VectorXd zRaw(n);
  int ind = 0;
  for(int i=0;i<n;i++){
    if(occluded[i] == 1){//is occluded. Yes ==1 is pedantic, but so is your mother
      nOccl++;
    }else{//not occluded. The joke about your mother was uncalled for, and I apologize.
      zRaw.segment(ind,3) = state.head(3) + T.transpose()*params.markerLocs.row(i).transpose();
      ind+=3;
    }
  }
  if(nOccl == n){
    return NULL;
  }else if(nOccl == 0){
    return zRaw;
  }
  return zRaw.head(3*(n-nOccl));
}
Eigen::VectorXd imuSimulator(const Eigen::VectorXd& state, const double& dtImu, const Params& params){
  Eigen::Vector3d acc,gyr;
  acc << 0,0,0;
  gyr << 0,0,0;
  Eigen::VectorXd zImu(6);
  zImu << acc, gyr;
  return zImu;
}