#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

#define DEBUG false

UKF::UKF() {
  is_initialized_ = false;
	
  time_us_ = 0;
	
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;

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

  /*
   State dimensions for constant turn rate and velocity magnitude model (CTRV)
   
       [  px ]
       [  px ]
   x = [  v  ]
       [  ψ  ]
       [ ​ ψ˙ ]
   
   where:
     px = position x
     py = position y
     v  = velocity magnitude
     ψ  = yaw angle (psi)
     ψ˙ = yaw angle rate (psi-dot)
   */
  
  // State dimensions
  n_x_ = 5;

	// augmented state dimentions
  n_aug_ = n_x_ + 2;
	
  // initial state vector
  x_ = VectorXd(n_x_);
  x_.fill(0.0);

  x_aug_ = VectorXd(n_aug_);
  x_aug_.fill(0.0);
	
  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  P_.fill(0.0);
	
  // process covariance matrix
  Q_ = MatrixXd(2, 2);
  Q_ << pow(std_a_, 2), 0,
        0, pow(std_yawdd_, 2);

  P_aug_ = MatrixXd(n_aug_, n_aug_);
  P_aug_.fill(0.0);
  P_aug_.topLeftCorner(n_x_, n_x_) = P_;
  P_aug_.bottomRightCorner(2, 2)   = Q_;

  // Augmented state dimension
  /* state + independent noise processes (independent meaning doesn't express their effect on state vector and they are independent of Δt)
            [  px     ]
            [  px     ]
   x(a,k) = [  v      ]
            [  ψ      ]
            [ ​ ψ˙     ]
            [ ​ ν(​a)   ]
            [ ​ ν(ψ˙˙) ]
   
   
   where:
      ν(​a)   = mu(a); longtitudial acceleration noise
      ν(ψ˙˙) = mu(ν(ψ˙˙)); mu-psi-dot-dot; yaw acceleration noise
  */
	
  // Sigma point spreading parameter
  lambda_ = 3 - n_x_;
	
  // Augmented sigma points
  n_aug_sigma_points_ = 2 * n_aug_ + 1;
	
  ///* predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, n_aug_sigma_points_);
  Xsig_pred_.fill(0.0);

  Xsig_aug_   = MatrixXd(n_aug_, n_aug_sigma_points_);
  Xsig_aug_.fill(0.0);

  // Weights of sigma points
  weights_ = VectorXd(n_aug_sigma_points_);
  weights_(0) = double(lambda_ / (lambda_ + n_aug_));
  for(int i = 1; i < n_aug_sigma_points_; i++) {
    weights_(i) = double(0.5 / (lambda_ + n_aug_));
  }

  // lidar measurement covariance matrix ...
  R_lidar_ = MatrixXd(2,2);
  R_lidar_ << pow(std_laspx_, 2), 0.0,
              0.0, pow(std_laspy_, 2);

  // radar measurement covariance matrix
  R_radar_ = MatrixXd(3,3);
  R_radar_ << pow(std_radr_, 2), 0.0, 0.0,
              0.0, pow(std_radphi_, 2), 0.0,
              0.0, 0.0, pow(std_radrd_, 2);


  // the current NIS for radar
  NIS_radar_ = 0;

  // the current NIS for laser
  NIS_laser_ = 0;

  p95thperc_2deg   = 5.991;
  p95thperc_3deg   = 7.815;
}

UKF::~UKF() {}

void UKF::Initialize(MeasurementPackage measurement_pack) {
  cout << "Initialize ..." << endl;
	
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;
  
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    VectorXd radar  = measurement_pack.raw_measurements_;

    // where:
    // radar[0]      = rho (ρ); radian distance from origin
    // radar[1]      = phi (ϕ); bearing
    // radar[2]      = rho_dot (ρ˙); Radial velocity

    double rho      = radar[0];
    double phi      = radar[1];
    double rho_dot  = radar[2];

    // Convert radar from polar to cartesian coordinates and initialize state.
    double px       = cos(phi) * rho;       // cos(phi) = adj/hyp = x / rho
    double py       = sin(phi) * rho;       // sin(phi) = opp/hyp = y / rho
    double vx       = cos(phi) * rho_dot;   // cos(phi) = adj/hyp = vx / rho_dot
    double vy       = sin(phi) * rho_dot;   // sin(phi) = opp/hyp = vy / rho_dot
    double v	      = sqrt(pow(vx, 2) + pow(vy, 2)); // velocity magnitude given vx, vy

    // If initial values are zero then increase uncertainty.
    if (fabs(px) < 0.001) {
      px = 1;
      P_(0,0) = 100;      // increase the uncertainty
    }

    if (fabs(py) < 0.001) {
      py = 1;
      P_(1,1) = 100;      // increase the uncertainty
    }

    x_ << px, py, v, 0.0, 0.0;

    is_initialized_ = true;
  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    VectorXd lidar = measurement_pack.raw_measurements_;
    // where:
    // lidar[0] = x position
    // lidar[1] = y position

    double px  = lidar[0];
    double py  = lidar[1];

    if(fabs(px) < 0.001) {
      px = 0.0;
      P_(0,0) = 100;    // increase the uncertainty
    }

    if(fabs(py) < 0.001) {
      py = 0.0;
      P_(1,1) = 100;    // increase the uncertainty
    }

    x_ << px, py, 0.0, 0.0, 0.0;

    is_initialized_ = true;
  } else {
    std::cerr << "Unhandled sensor_type == " << measurement_pack.sensor_type_ << std::endl;
  }
	
  x_aug_ << x_.array(), 0.0, 0.0;
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage measurement_pack) {
  if( !is_initialized_ ) {
    Initialize(measurement_pack);
    time_us_ = measurement_pack.timestamp_;
    return;
  }
  
  // Compute the time elapsed between the current and previous measurements
  double dt = (measurement_pack.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds
  time_us_ = measurement_pack.timestamp_;
  
  // prediction step ...
  Prediction(dt);
  
  // update step ...
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    if( use_radar_ ) {
      UpdateRadar(measurement_pack);
    }
  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    if( use_laser_ ) {
      UpdateLidar(measurement_pack);
    }
  } else {
    std::cerr << "Unhandled sensor_type == " << measurement_pack.sensor_type_ << std::endl;
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  // State prediction
  //		- create augmented mean state (x_aug)
  //		- create augmented covariance matrix (P_)
  //		- create augmented sigma points (Xsig_aug_)
  //		- predict sigma points (Xsig_pred_)
  //		- predict state mean (x_)
  //		- predict covariance matrix (P_)
  
  // create augmented mean state
  x_aug_.fill(0.0);
  x_aug_.head(n_x_) = x_;
  
  // create augmented covariance matrix
  P_aug_.fill(0.0);
  P_aug_.topLeftCorner(n_x_, n_x_) = P_;
  P_aug_.bottomRightCorner(2, 2) = Q_;
  
  // create square root matrix
  MatrixXd L = P_aug_.llt().matrixL();
  
  // create augmented sigma points
  Xsig_aug_.col(0)  = x_aug_;
  for (int i = 0; i < n_aug_; i++)
  {
    Xsig_aug_.col(i + 1)          = x_aug_ + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug_.col(i + 1 + n_aug_) = x_aug_ - sqrt(lambda_ + n_aug_) * L.col(i);
  }
  
  // predict sigma points
  for (int i = 0; i< n_aug_sigma_points_; i++)
  {
    // extract values for better readability
    double p_x = Xsig_aug_(0,i);
    double p_y = Xsig_aug_(1,i);
    double v = Xsig_aug_(2,i);
    double yaw = Xsig_aug_(3,i);
    double yawd = Xsig_aug_(4,i);
    double nu_a = Xsig_aug_(5,i);
    double nu_yawdd = Xsig_aug_(6,i);
    
    // predicted state values
    double px_p, py_p;
    
    // avoid division by zero
    if (fabs(yawd) > 0.001) {
      px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
      py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    } else {
      px_p = p_x + v*delta_t*cos(yaw);
      py_p = p_y + v*delta_t*sin(yaw);
    }
    
    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;
    
    // add noise
    px_p = px_p + 0.5 * pow(delta_t, 2) * cos(yaw) * nu_a;
    py_p = py_p + 0.5 * pow(delta_t, 2) * sin(yaw) * nu_a;
    v_p = v_p + nu_a * delta_t;
    
    yaw_p = yaw_p + 0.5 * pow(delta_t, 2) * nu_yawdd;
    yawd_p = yawd_p + delta_t * nu_yawdd;
    
    // write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }
  
  // predict state mean
  x_.fill(0.0);
  for(int i = 0; i < n_aug_sigma_points_; i++) {
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }
  
  // predict state covariance matrix
  P_.fill(0.0);
  for(int i = 0; i < n_aug_sigma_points_; i++) {
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    
    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage measurement_pack) {
  // set measurement dimension, lidar can measure px, py
  int n_z = 2;
  
  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, n_aug_sigma_points_);
  Zsig.fill(0.0);
  
  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  
  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  
  // transform sigma points into measurement space
  for (int i=0; i< n_aug_sigma_points_; i++) {
    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);
    
    Zsig(0, i) = px;
    Zsig(1, i) = py;
  }
  
  // calculate mean predicted measurement
  for (int i=0; i< n_aug_sigma_points_; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
  
  // calculate measurement covariance matrix S
  for (int i=0; i < n_aug_sigma_points_; i++) {
    MatrixXd z_diff = Zsig.col(i) - z_pred;
    S = S + weights_(i) * (z_diff * z_diff.transpose());
  }
  
  // add measurement noise covariance matrix
  S = S + R_lidar_;
  
  // incoming laser measurement
  VectorXd z = measurement_pack.raw_measurements_;
  
  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  
  //calculate cross correlation matrix
  for(int i = 0; i < 2 * n_aug_ + 1; i++) {
    MatrixXd z_diff = Zsig.col(i) - z_pred;
    MatrixXd x_diff = Xsig_pred_.col(i) - x_;
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  
  // calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  
  // update state mean and covariance matrix
  VectorXd z_diff = z - z_pred;
  
  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
  
  // Radar NIS ...
  NIS_laser_  = z_diff.transpose() * S.inverse() * z_diff;
  
  if( DEBUG ) {
    DumpMatrix("z", z);
    DumpMatrix("z_pred", z_pred);
    DumpMatrix("x_", x_);
    DumpMatrix("P_", P_);
  }
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage measurement_pack) {
  // set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;
  
  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, n_aug_sigma_points_);
  Zsig.fill(0.0);
  
  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  
  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  
  // transform sigma points into measurement space
  for (int i=0; i< n_aug_sigma_points_; i++) {
    double px    = Xsig_pred_(0,i);
    double py    = Xsig_pred_(1,i);
    double v     = Xsig_pred_(2,i);
    double yaw   = Xsig_pred_(3,i);
    double yawdd = Xsig_pred_(4,i);
    
    double vx = cos(yaw) * v;
    double vy = sin(yaw) * v;
    
    double rho = sqrt(pow(px, 2) + pow(py, 2));																		// Radian distance from origin
    double phi = fabs(px) < 0.001 || abs(px) < 0.001 ? 0 : atan2(py, px);         // bearing; tan(ϕ) = opp/adj = phi; arctan(phi) = phi
    double rho_dot  = fabs(rho) > 0.001 ? (px * vx + py * vy) / rho : 0.0;				// Radial velocity
    
    Zsig(0, i) = rho;
    Zsig(1, i) = phi;
    Zsig(2, i) = rho_dot;
  }
  
  // calculate mean predicted measurement
  for (int i=0; i< n_aug_sigma_points_; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
  
  // calculate measurement covariance matrix S
  for (int i=0; i < n_aug_sigma_points_; i++) {
    MatrixXd z_diff = Zsig.col(i) - z_pred;
    
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    
    S = S + weights_(i) * (z_diff * z_diff.transpose());
  }
  
  // add measurement noise covariance matrix
  S = S + R_radar_;
  
  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  
  //calculate cross correlation matrix
  for(int i = 0; i < n_aug_sigma_points_; i++) {
    MatrixXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    
    MatrixXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    
    
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  
  // calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  
  // incoming radar measurement
  VectorXd z = measurement_pack.raw_measurements_;
  
  // update state mean and covariance matrix
  VectorXd z_diff = z - z_pred;
  
  // angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
  
  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
  
  // Radar NIS ...
  NIS_radar_  = z_diff.transpose() * S.inverse() * z_diff;
}

void UKF::DumpMatrix(std::string name, const MatrixXd &x) {
  std::cout << "Matrix dumper: " << std::endl;
  std::cout << name << " = " << std::endl << x << std::endl;
  std::cout << std::endl;
}
