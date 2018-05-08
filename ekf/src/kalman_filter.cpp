#include "kalman_filter.h"
#define PI 3.14159265

#include <iostream>
using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init( VectorXd &x_in, 
    MatrixXd &P_in, 
    MatrixXd &F_in,
    MatrixXd &H_in, 
    MatrixXd &Hj_in, 
    MatrixXd &R_in, 
    MatrixXd &R_ekf_in, 
    MatrixXd &Q_in ) 
{
  cout << "In KalmanFilter::Init" << endl;
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  Hj_ = Hj_in;
  // checking if eigen does a deep or shallow copy with
  // operator=.
  // cout << &Hj_(0) << endl;
  // cout << &Hj_in(0) << endl;
  // The two printed values are different so it appears to do a deep copy.
  // A deep copy is the desired behavior, because when I call Init in 
  // FusionEKF.cpp, some of the arguments have local scope.
  R_ = R_in;
  R_ekf_ = R_ekf_in;
  Q_ = Q_in;
  I_ = Eigen::MatrixXd::Identity(4,4);
}

void KalmanFilter::Predict() 
{
  // predict the state
  x_ = F_*x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_*P_*Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) 
{
  // Update the state using Kalman Filter equations
  VectorXd y = z - H_*x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_*P_*Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_*Ht*Si;

  // New state
  x_ = x_ + ( K*y );
  P_ = ( I_ - K*H_ )*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) 
{
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];

  // If rho == 0, skip the update step to avoid dividing by zero.
  // This is crude but should be fairly robust on our data set.
  if( px == 0. && py == 0. )
    return;

  Hj_ = tools.CalculateJacobian( x_ );
  VectorXd hofx(3);
  float rho = sqrt( px*px + py*py );
  hofx << rho, atan2( py, px ), ( px*vx + py*vy )/rho;

  // Update the state using Extended Kalman Filter equations
  VectorXd y = z - hofx;
  if( y[1] > PI )
    y[1] -= 2.f*PI;
  if( y[1] < -PI )
    y[1] += 2.f*PI;
  MatrixXd Hjt = Hj_.transpose();
  MatrixXd S = Hj_*P_*Hjt + R_ekf_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_*Hjt*Si;

  // Compute new state
  x_ = x_ + ( K*y );
  P_ = ( I_ - K*Hj_ )*P_;
}
