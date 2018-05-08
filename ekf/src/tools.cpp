#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() : mse(4), 
    lastmse(4),
    residual(4),
    kahanerror(4),
    rmse(4)
{
  resetRMSE();
}

Tools::~Tools() {}

void Tools::resetRMSE()
{
  mse << 0, 0, 0, 0;
  lastmse << 0, 0, 0, 0;
  residual << 0, 0, 0, 0;
  kahanerror << 0, 0, 0, 0;
}

// Calculate the RMSE
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) 
{
  float t = estimations.size(); // Current timestep index

  // check the validity of the inputs
  if( t == 0 )
    cout << "Error in CalculateRMSE:  estimations.size() = 0" << endl;
  if( t != ground_truth.size() )
    cout << "Error in CalculateRMSE: sizes of estimation and ground truth do not match" << endl;

  // Rather than recomputing from the entire estimations array,
  // I store the running error from the last timestep persistently.
  // This ensures that CalculateRMSE() remains O(1) instead of O(N)
  // at the Nth timestep.
  // 
  // for(int i=0; i < estimations.size(); i++)
  // {

  // Recover the running sum of the error, rather than the running
  // mean, so we can add the current residual
  mse = mse*(t-1); 

  // Add the current residual using a Kahan sum to minimize floating-point
  // rounding error.
  residual = estimations[t-1] - ground_truth[t-1];
  residual = residual.array()*residual.array();
  residual += kahanerror;
  mse += residual; 
  kahanerror = residual - ( mse - lastmse ); 
  lastmse = mse;

  // }

  // Calculate the new mean
  mse = mse/estimations.size();

  // Calculate the RMSE
  rmse = mse.array().sqrt();

  // cout << estimations.size() << endl;
  if( rmse(0) > .11 ||
      rmse(1) > .11 ||
      rmse(2) > .52 ||
      rmse(3) > .52 )
    cout << "Warning at timestep " << t << ":  rmse = " 
         << rmse(0) << "  " << rmse(1) << "  " 
         << rmse(2) << "  " << rmse(3) << endl
         << " currently exceeds tolerances of "
         << ".11, .11, .52, .52" << endl;

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) 
{
  // Calculate a Jacobian for the transformation from the state vector 
  // px, py, vx, vy to the radar measurement space
  // rho, phi, rhodot.
  

  // This should be copy-elided into wherever it's being returned.
  MatrixXd Hj(3,4);

  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //check division by zero
  if( px == 0 && py == 0 )
  {
    cout << "Error:  division by zero in CalculateJacobian" << endl;
    return Hj;
  }

  //compute the Jacobian 
  float rho = sqrt( px*px + py* py );
  float rho2 = rho*rho;
  float rho3 = rho2*rho;
  Hj <<                 px/rho,                    py/rho,      0,      0,
                      -py/rho2,                   px/rho2,      0,      0,
     py*( vx*py - vy*px )/rho3, px*( vy*px - vx*py )/rho3, px/rho, py/rho;

  return Hj;
}
