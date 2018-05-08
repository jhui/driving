#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

  // Open NIS data files
  NISvals_radar_.open( "../postprocess_NIS/NISvals_radar.txt", ios::out );
  NISvals_laser_.open( "../postprocess_NIS/NISvals_laser.txt", ios::out );

  // Check for errors opening the files
  if( !NISvals_radar_.is_open() )
  {
    cout << "Error opening NISvals_radar.txt" << endl;
    exit(1);
  }

  if( !NISvals_laser_.is_open() )
  {
    cout << "Error opening NISvals_laser.txt" << endl;
    exit(1);
  }

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3.;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1.;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  */
  
  n_x_ = 5;

  n_aug_ = 7;

  n_radar_ = 3;

  n_laser_ = 2;

  lambda_ = 3 - n_aug_;

  x_aug_ = VectorXd( n_aug_ );

  deltax_ = VectorXd( n_aug_ );

  Xsig_aug_ = MatrixXd( n_aug_, 2*n_aug_+1 );

  Xsig_pred_ = MatrixXd( n_x_, 2*n_aug_+1 );

  P_aug_ = MatrixXd( n_aug_, n_aug_ ); 

  L_ = MatrixXd( n_aug_, n_aug_ ); 

  weights_ = VectorXd( 2*n_aug_+1 );
  weights_(0) = lambda_/( lambda_ + n_aug_ );
  for( int i=1; i<2*n_aug_+1; i++ )  
     weights_(i) = 0.5/( n_aug_ + lambda_ );

  // Variables used for radar update
  z_pred_radar_ = VectorXd( n_radar_ );
  deltaz_radar_ = VectorXd( n_radar_ );
  Zsig_radar_ = MatrixXd( n_radar_, 2*n_aug_+1 );
  // Radar measurement noise covariance matrix is constant/persistent
  R_radar_ = MatrixXd( n_radar_, n_radar_ );
  R_radar_.fill(0.);
  R_radar_(0,0) = std_radr_*std_radr_;
  R_radar_(1,1) = std_radphi_*std_radphi_;
  R_radar_(2,2) = std_radrd_*std_radrd_;
  S_radar_ = MatrixXd( n_radar_, n_radar_ );
  Tc_radar_ = MatrixXd( n_x_, n_radar_ );
  K_radar_ = MatrixXd( n_x_, n_radar_ );

  // Variables used for laser update
  z_pred_laser_ = VectorXd( n_laser_ );
  deltaz_laser_ = VectorXd( n_laser_ );
  Zsig_laser_ = MatrixXd( n_laser_, 2*n_aug_+1 );
  // Laser measurement noise covariance matrix is constant/persistent
  R_laser_ = MatrixXd( n_laser_, n_laser_ );
  R_laser_.fill(0.);
  R_laser_(0,0) = std_laspx_*std_laspx_;
  R_laser_(1,1) = std_laspy_*std_laspy_;
  S_laser_ = MatrixXd( n_laser_, n_laser_ );
  Tc_laser_ = MatrixXd( n_x_, n_laser_ );
  K_laser_ = MatrixXd( n_x_, n_laser_ );

  // State covariance matrix P will be initialized using some data
  // from the first measurement, in ProcessMeasurement() below.
}

UKF::~UKF() 
{
  NISvals_radar_.close();
  NISvals_laser_.close();
}

/**
 * @param {MeasurementPackage} meas_pack The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement( MeasurementPackage meas_pack ) 
{
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  if (!is_initialized_) 
  {
    // Initialize the state ekf_.x_ with the first measurement.
    // Create the covariance matrix.

    cout << "Initializing unscented Kalman filter" << endl;

    if( meas_pack.sensor_type_ == MeasurementPackage::RADAR ) 
    {
      // Convert radar from polar to cartesian coordinates and initialize state.
      double rho = meas_pack.raw_measurements_[0];
      double phi = meas_pack.raw_measurements_[1];
      x_ << rho*cos(phi), rho*sin(phi), 0., 0., 0.;
    }
    else if( meas_pack.sensor_type_ == MeasurementPackage::LASER ) 
    {
      // Initialize state.
      x_ << meas_pack.raw_measurements_[0], meas_pack.raw_measurements_[1], 0., 0., 0.;
    }

    previous_timestamp_ = meas_pack.timestamp_;

    // Estimate the initial state covariance matrix
    // with a moderate covariance of 1 for all values.  This is somewhat 
    // unrealistic because the velocity, yaw angle, and yaw rate are completely
    // unknown from the initial measurements.
    P_.fill(0.);
    P_(0,0) = 1.;
    P_(1,1) = 1.;
    P_(2,2) = 1.;
    P_(3,3) = 1.;
    P_(4,4) = 1.;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  // Compute the time from the previous measurement in seconds.
  double dt = ( meas_pack.timestamp_ - previous_timestamp_ )/1000000.0;
	previous_timestamp_ = meas_pack.timestamp_;

  if( dt > 0.0001 )
    Prediction( dt );

  if( meas_pack.sensor_type_ == MeasurementPackage::RADAR ) 
  {
    // Radar update
    UpdateRadar( meas_pack );
  } 
  else 
  {
    // Laser update
    UpdateLidar( meas_pack );
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
// Much of this code is taken from previously completed quizzes.
void UKF::Prediction(double delta_t) 
{
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  //create augmented mean state
  x_aug_.head(5) = x_;
  x_aug_(5) = 0;
  x_aug_(6) = 0;
 
  //create augmented covariance matrix
  P_aug_.fill(0.);


  P_aug_.topLeftCorner(5,5) = P_;
  P_aug_(5,5) = std_a_*std_a_;
  P_aug_(6,6) = std_yawdd_*std_yawdd_;
  
  //create square root matrix
  L_ = P_aug_.llt().matrixL();
  
  //create augmented sigma points
  Xsig_aug_.col(0) = x_aug_;
  for( int i = 0; i < n_aug_; i++)
  {
    Xsig_aug_.col(i+1)       = x_aug_ + sqrt(lambda_+n_aug_)*L_.col(i);
    Xsig_aug_.col(i+n_aug_+1) = x_aug_ - sqrt(lambda_+n_aug_)*L_.col(i);
  }

  // Run each augmented sigma points through the process model with noise
  // (prediction step)
  double dt = delta_t;
  double dt2 = delta_t*delta_t;
  for( int pt = 0; pt < 2*n_aug_+1; pt++ )
  {
      double x       = Xsig_aug_(0,pt);
      double y       = Xsig_aug_(1,pt);
      double v       = Xsig_aug_(2,pt);
      double psi     = Xsig_aug_(3,pt);
      double psid    = Xsig_aug_(4,pt);
      double nua     = Xsig_aug_(5,pt);
      double nupsidd = Xsig_aug_(6,pt);
      
      if( psid == 0. )
      {
        Xsig_pred_(0,pt) = x + v*cos(psi)*dt + 0.5f*dt2*cos(psi)*nua;
        Xsig_pred_(1,pt) = y + v*sin(psi)*dt + 0.5f*dt2*sin(psi)*nua;
      }
      else
      {
        Xsig_pred_(0,pt) = x + ( v/psid )*( sin(psi + psid*dt) - sin(psi) ) 
                          + 0.5f*dt2*cos(psi)*nua;
        Xsig_pred_(1,pt) = y + ( v/psid )*( -cos(psi + psid*dt) + cos(psi) ) 
                          + 0.5f*dt2*sin(psi)*nua;
      }    
      Xsig_pred_(2,pt) = v    +           dt*nua;
      Xsig_pred_(3,pt) = psi  + psid*dt + 0.5f*dt2*nupsidd;
      Xsig_pred_(4,pt) = psid +           dt*nupsidd;
  }

  //predict state mean
  x_.fill(0.);
  for( int i = 0; i < 2*n_aug_+1; i++ )
    x_ = x_ + weights_(i)*Xsig_pred_.col(i);

  //predict state covariance matrix
  P_.fill(0.);
  for( int i = 0; i < 2*n_aug_+1; i++)
  {
    deltax_ = Xsig_pred_.col(i) - x_;
    while( deltax_(3) > M_PI ) deltax_(3) -= 2.*M_PI;
    while( deltax_(3) < -M_PI ) deltax_(3) += 2.*M_PI;
    // outer product
    // https://eigen.tuxfamily.org/dox/group__QuickRefPage.html#matrixonly
    P_ = P_ + weights_(i)*deltax_*deltax_.transpose();   
  }  
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_pack
 */
// Much of this code is taken from previously completed quizzes.
void UKF::UpdateRadar(MeasurementPackage meas_pack) 
{
  // Transform sigma points into measurement space
  for( int pt = 0; pt < 2*n_aug_ + 1; pt++ )
  {
    double px = Xsig_pred_(0,pt);
    double py = Xsig_pred_(1,pt);
    double v = Xsig_pred_(2,pt);
    double psi = Xsig_pred_(3,pt);
    Zsig_radar_(0,pt) = sqrt(px*px+py*py);
    Zsig_radar_(1,pt) = atan2(py,px);
    Zsig_radar_(2,pt) = ( px*cos(psi)*v + py*sin(psi)*v )/sqrt(px*px+py*py);
  }
  
  //calculate mean predicted measurement
  z_pred_radar_.fill(0.);
  for( int pt = 0; pt < 2*n_aug_ + 1; pt++ )
  {
    z_pred_radar_ = z_pred_radar_ + weights_(pt)*Zsig_radar_.col(pt);
  }
  
  while( z_pred_radar_(1) > M_PI ) z_pred_radar_(1)-=2.*M_PI;
  while( z_pred_radar_(1) <-M_PI ) z_pred_radar_(1)+=2.*M_PI;
  
  //calculate measurement covariance matrix S_radar_
  S_radar_.fill(0.);
  for( int pt = 0; pt < 2*n_aug_ + 1; pt++ )
  {
    deltaz_radar_ = Zsig_radar_.col(pt) - z_pred_radar_;
    while(deltaz_radar_(1)> M_PI) deltaz_radar_(1)-=2.*M_PI;
    while(deltaz_radar_(1)<-M_PI) deltaz_radar_(1)+=2.*M_PI;
    S_radar_ = S_radar_ + weights_(pt)*deltaz_radar_*deltaz_radar_.transpose();
  }

  S_radar_ = S_radar_ + R_radar_;

  //calculate cross correlation matrix
  Tc_radar_.fill(0.);
  for( int pt = 0; pt < 2*n_aug_ + 1; pt++ )
  {
      deltax_ = Xsig_pred_.col(pt) - x_;
      deltaz_radar_ = Zsig_radar_.col(pt) - z_pred_radar_;
      while( deltax_(1)> M_PI ) deltax_(1)-=2.*M_PI;
      while( deltax_(1)<-M_PI ) deltax_(1)+=2.*M_PI;
      while( deltaz_radar_(1)> M_PI ) deltaz_radar_(1)-=2.*M_PI;
      while( deltaz_radar_(1)<-M_PI ) deltaz_radar_(1)+=2.*M_PI;
      Tc_radar_ = Tc_radar_ + weights_(pt)*deltax_*deltaz_radar_.transpose();
  }
  
  //calculate K_radar_alman gain K_radar_;
  K_radar_ = Tc_radar_*S_radar_.inverse();
  
  //update state mean and covariance matrix
  deltaz_radar_ = meas_pack.raw_measurements_ - z_pred_radar_;
  while( deltaz_radar_(1) > M_PI ) deltaz_radar_(1)-=2.*M_PI;
  while( deltaz_radar_(1) <-M_PI ) deltaz_radar_(1)+=2.*M_PI;
  
  x_ = x_ + K_radar_*deltaz_radar_;
  P_ = P_ - K_radar_*S_radar_*K_radar_.transpose();

  // Uncomment the following to print normalized innovation squared (NIS_radar_),
  // so that it can be plotted and serve as a consistency check on 
  // our choice of process noise values
  NIS_radar_ = deltaz_radar_.transpose()*S_radar_.inverse()*deltaz_radar_;
  NISvals_radar_ << NIS_radar_ << endl;;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_pack
 */
void UKF::UpdateLidar(MeasurementPackage meas_pack) 
{
  // Transform sigma points into measurement space
  for( int pt = 0; pt < 2*n_aug_ + 1; pt++ )
  {
    Zsig_laser_(0,pt) = Xsig_pred_(0,pt);
    Zsig_laser_(1,pt) = Xsig_pred_(1,pt);
  }
  
  //calculate mean predicted measurement
  z_pred_laser_.fill(0.);
  for( int pt = 0; pt < 2*n_aug_ + 1; pt++ )
    z_pred_laser_ = z_pred_laser_ + weights_(pt)*Zsig_laser_.col(pt);
  
  //calculate measurement covariance matrix S_laser_
  S_laser_.fill(0.);
  for( int pt = 0; pt < 2*n_aug_ + 1; pt++ )
  {
    deltaz_laser_ = Zsig_laser_.col(pt) - z_pred_laser_;
    S_laser_ = S_laser_ + weights_(pt)*deltaz_laser_*deltaz_laser_.transpose();
  }

  S_laser_ = S_laser_ + R_laser_;

  //calculate cross correlation matrix
  Tc_laser_.fill(0.);
  for( int pt = 0; pt < 2*n_aug_ + 1; pt++ )
  {
      deltax_ = Xsig_pred_.col(pt) - x_;
      deltaz_laser_ = Zsig_laser_.col(pt) - z_pred_laser_;
      Tc_laser_ = Tc_laser_ + weights_(pt)*deltax_*deltaz_laser_.transpose();
  }
  
  //calculate K_laser_alman gain K_laser_;
  K_laser_ = Tc_laser_*S_laser_.inverse();
  
  //update state mean and covariance matrix
  deltaz_laser_ = meas_pack.raw_measurements_ - z_pred_laser_;
  
  x_ = x_ + K_laser_*deltaz_laser_;
  P_ = P_ - K_laser_*S_laser_*K_laser_.transpose();

  // Uncomment the following to print normalized innovation squared (NIS_laser_),
  // so that it can be plotted and serve as a consistency check on 
  // our choice of process noise values
  NIS_laser_ = deltaz_laser_.transpose()*S_laser_.inverse()*deltaz_laser_;
  NISvals_laser_ << NIS_laser_ << endl;
}
