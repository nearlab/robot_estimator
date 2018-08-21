#include "estimator.h"

Estimator::Estimator(){
  this->params = Params();
  this->isInitialized = false;
  this->state = Eigen::VectorXd(13);
  this->P = Eigen::MatrixXd(12,12);
}
void Estimator::predict(const Eigen::VectorXd& zImu, const double& dtImu){
  Eigen::VectorXd r(3),v(3),q(4),ba(3);
  double dt = dtImu;
  r = state.segment(0,3);
  v = state.segment(7,3);
  q = state.segment(3,4);
  ba = state.segment(10,3);
  Eigen::Matrix3d T = quat2rot(q);
  Eigen::Matrix3d Tt = T.transpose();
  Eigen::Vector3d wk = zImu.tail(3);
  Eigen::VectorXd dq(4);
  dq.head(3) = dt*wk/2;
  dq.tail(1) = sqrt(1-pow((dt*wk/2).norm(),2));
  Eigen::Vector3d ak = zImu.head(3) - ba - T*(ColVectorXd(3) << 0,0,9.8).finished();
  Eigen::Matrix3d wkx,akx;
  wkx = crossProductEquivalent(wk);
  akx = crossProductEquivalent(ak);
  
  Eigen::VectorXd rk(3),vk(3),qk(4),bak(3);
  double dt = dtImu;
  rk = r + v*dt + 1/2*Tt*ak*pow(dt,2) - 1/6*Tt*akx*wk*pow(dt,3);
  vk = v + ak*dt - 1/2*Tt*akx*wk*pow(dt,2);
  qk = quatRot(q,dq);
  qk.head(2) << 0,0;
  qk = qk/qk.norm();
  bak = ba;

  state.segment(0,3) = rk;
  state.segment(7,3) = vk;
  state.segment(3,4) = qk;
  state.segment(10,3) = bak;

  //Update error covariance
  Eigen::Matrix3d wxax = crossProductEquivalent(wkx*ak);
  Eigen::MatrixXd F,M;
  F = Eigen::MatrixXd(12,12);
  M = Eigen::MatrixXd(12,9);
  Eigen::Matrix3d z3 = Eigen::MatrixXd::Zero(3,3);
  Eigen::Matrix3d i3 = Eigen::MatrixXd::Identity(3,3);
  F << i3, pow(dt,3)/6*Tt*wxax-pow(dt,2)/2*Tt*akx, dt*i3, pow(dt,3)/6*Tt*wkx-pow(dt,2)/2*Tt, 
       z3, quat2rot(dq)                          , z3   , z3,
       z3, pow(dt,2)/2*Tt*wxax-dt*Tt*akx         , i3   , pow(dt,2)*Tt*wkx-dt*Tt,
       z3, z3                                    , z3   , i3;

  M << pow(dt,3)*Tt*wkx-pow(dt,2)*Tt,-pow(dt,3)/6*Tt*akx,z3,
       z3                           ,-dt*i3             ,z3,
       pow(dt,2)/2*Tt*wkx-dt*Tt     ,-pow(dt,2)/2*Tt*akx,z3,
       z3                           ,z3                 ,i3;

  this->P = F*this->P*F.transpose() + M*this->Q*M.transpose();
}
void Estimator::correct(const Eigen::VectorXd& zMarkers, const double& dtMarkers){
  Eigen::MatrixXd HMarkers = parseMeasMarkers(zMarkers);
  if(HMarkers == NULL){
    return;
  }
  Eigen::MatrixXd H = HMarkers;//Could potentially add in more measurements
  Eigen::MatrixXd R = this->params.RMarkers;
  Eigen::MatrixXd Y = H*this->P*H.transpose() + R;
  Eigen::MatrixXd C = this->P*H.transpose();
  Eigen::MatrixXd K = C*Y.inverse();
  Eigen::VectorXd dzMarkers = (zMarkers - markerSimulator(this->state,this->params));
  Eigen::VectorXd zHatError = dzMarkers;
  Eigen::VectorXd dx = K*zHatError;
  this->state.segment(0,3) += dx.segment(1:3);
  Eigen::VectorXd dq(4);
  dq.head(3) = dx.segment(3,4);
  dq.tail(1) = sqrt(1-pow((dt*dx.segment(3,3)/2).norm(),2));
  this->state.segment(3,4) = quatRot(this->state.segment(3,3),dq);
  this->state.segment(3,4) /= this->state.segment(3,4).norm();
  this->state.segment(7,6) += dx.segment(6,6);
  this->P = (Eigen::MatrixXd::Identity(12,12)-K*H)*this->P*(Eigen::MatrixXd::Identity(12,12)-K*H).transpose()+K*R*K.transpose();
  this->P = .5*(this->P + (this->P).transpose());
}
Eigen::MatrixXd Estimator::parseMeasMarkers(const Eigen::VectorXd& zMarkersRaw){
  //Returns the Jacobian for the measurement equation for the Vicon Markers
  double n = size(zMarkersRaw);
  double[5] occluded;
  nOccl = 0;
  for(i=0;i<n*3;i++){
    if(isnan(zMarkersRaw(i))){
      int ind = ceil(i/3)
      if(occluded[ind]==0){
        nOccl = nOccl+1;
        occluded[ind] = 1;
      }
    }
  }
  if(3*(n-nOccl)==0){
    return NULL;
  }
  Eigen::VectorXd zMarkers(3*(n-nOccl));
  if(nOccl>0){
    indTemp = 1;
    for(i=0;i<n;i++){
      if(occluded[i]==0){
        zMarkers.block(indTemp,3) = zMarkersRaw.block(i,3);
        indTemp+=3;
      }
    }
  }else{
    zMarkers = zMarkersRaw;
  }
  double qx = this->state(4);
  double qy = this->state(5);
  double qz = this->state(6);
  dzdqx <<  0    , 2*qy , 2*qz ,
            2*qy , -4*qx, -2*qw,
            2*qz , 2*qw , -4*qx;

  dzdqx <<  -4*qy, 2*qx , 2*qw ,
            2*qx , 0    , 2*qz ,
            -2*qw, 2*qz , -4*qy;

  dzdqx <<  -4*qz, 2*qw , -2*qx,
            2*qw , -4*qz, 2*qy ,
            2*qx , 2*qy , 0    ;
  Eigen::MatrixXd H(3*(n-nOccl),12);
  rMarker = this->params.markerLocs;
  for(i=0;i<n-nOccl;i++){
    H.block(i,0,3,3) = MatrixXd::Identity(3,3);
    H.block(i,3,3,1) = dzdqx*(rMarker.row(i).transpose());
    H.block(i,4,3,1) = dzdqy*(rMarker.row(i).transpose());
    H.block(i,5,3,1) = dzdqz*(rMarker.row(i).transpose());
    H.block(i,6,3,6).setZero();
  }
  return H;
}
void Estimator::estimateStateFromMarkers(const Eigen::VectorXd& zMarkers){
  Eigen::VectorXd x(7);
  Eigen::VectorXd dx(7);
  int nMarkers = zMarkers.size()/3;
  int count = 0;
  double sf = 2;
  double lambda = 1;
  bool firstRun = true;
  cost = (zMarker - markerSimulator(x,this->params)).norm();
  while(dx.norm()>.00001 && count<1000){
    double qx = x(4);
    double qy = x(5);
    double qz = x(6);
    double qw = x(7);
    Eigen::Matrix3d dzdqx,dzdqy,dzdqz,dzdqw;
    dzdqx << 0    , 2*qy , 2*qz ,
             2*qy , -4*qx, -2*qw,
             2*qz , 2*qw , -4*qx;

    dzdqy << -4*qy, 2*qx , 2*qw ,
             2*qx , 0    , 2*qz ,
             -2*qw, 2*qz , -4*qy;

    dzdqz << -4*qz, 2*qw , -2*qx,
             2*qw , -4*qz, 2*qy ,
             2*qx , 2*qy , 0    ;

    dzdqw << 4*qw , -2*qz, 2*qy ,
             2*qz , 4*qw , -2*qx,
             -2*qy, 2*qx , 4*qw ;

    Eigen::MatrixXd H(2*nMarkers,7); //Jacobian of measurement equation
    rMarker = this->params.markerLocs;
    for(i=0;i<nMarkers;i++){
      H.block(i,0,3,3) = MatrixXd::Identity(3,3);
      H.block(i,3,3,1) = dzdqx*(rMarker.row(i).transpose());
      H.block(i,4,3,1) = dzdqy*(rMarker.row(i).transpose());
      H.block(i,5,3,1) = dzdqz*(rMarker.row(i).transpose());
      H.block(i,6,3,1) = dzdqw*(rMarker.row(i).transpose());
    }
    Eigen::MatrixXd Ht = H.transpose();
    //Determine "optimal" scaling factor. The essence of Levenberg-Marquard
    //Lower scaling factor means larger step size
    while(firstRun){
      Eigen::VectorXd dJdx1 = (Ht*H+lambda*(Ht*H).diagonal().asDiagonal()).inverse()*Ht*cost;
      Eigen::VectorXd = x + dJdx1;
      x1.tail(4) = x1.tail(4)/x1.tail(4).norm();
      double cost1 = (zMarkers - markerSimulator(x1,this->params)).norm();

      Eigen::VectorXd dJdx2 = (Ht*H+lambda/sf*(Ht*H).diagonal().asDiagonal()).inverse()*Ht*cost;
      Eigen::VectorXd = x + dJdx2;
      x2.tail(4) = x2.tail(4)/x1.tail(4).norm();
      double cost2 = (zMarkers - markerSimulator(x2,this->params)).norm();

      if(cost1<cost2&&cost1<cost){
        firstRun = false;
      }else if(cost2<cost1&&cost2<cost){
        firstRun = false;
        lambda = lambda/2;
      }else{
        lambda = lambda*sf*sf; //Square it to be more efficient with checking, since we're just dividing next time anyway
      }
      JacobiSVD<MatrixXd> svd(A);
      double rcond = svd.singularValues()(0)/svd.singularValues()(svd.singularValues().size()-1);
      if(rcond<1e-15){// If there isn't a step size small enough that offers any improvements, return.
        return;
      }
    }
    Eigen::VectorXd dJdx = (Ht*H+lambda*(Ht*H).diagonal().asDiagonal()).inverse()*Ht*(zMarkers - markerSimulator(x,this->params);
    Eigen::VectorXd = x + dJdx;
    x.tail(4) = x.tail(4)/x.tail(4).norm();
    count++;
  }
  this->state.head(7) = x;
  isInitialized = true;
}
Eigen::VectorXd Estimator::getState(){
  return this->state;
}
Eigen::MatrixXd Estimator::getCovariance(){
  return this->P;
}
Params Estimator::getParams(){
  return this->params;
}
bool Estimator::initialized(){
  return this->isInitialized;
}
