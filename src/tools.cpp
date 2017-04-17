#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) {
  // Calculate the RMSE
  
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if ( estimations.size() == 0 || estimations.size() != ground_truth.size() ) {
    return rmse;
  }
  
  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i){
    VectorXd est = estimations[i];
    VectorXd actual = ground_truth[i];
    VectorXd delta = est - actual;
    VectorXd delta_sq = delta.array() * delta.array();
    rmse = rmse + delta_sq;
  }
  
  //calculate the mean
  rmse = rmse / (double)estimations.size();
  
  //calculate the squared root
  rmse = rmse.array().sqrt();
  
  //return the result
  return rmse;
}
