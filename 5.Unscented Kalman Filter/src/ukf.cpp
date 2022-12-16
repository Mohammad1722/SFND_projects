#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

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
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  is_initialized_ = false;
  n_x_ = 5;
  n_aug_ = 7;
  
  // Spreading parameter
  lambda_ = 0;
  
  // Sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  
  // Weights vector
  weights_ = VectorXd(2*n_aug_ + 1);
  
  // Start time
  time_us_ = 0;
  
  // Set NIS
  NIS_radar_ = 0;
  NIS_laser_ = 0;
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  if (!is_initialized_) 
  {
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) 
    {
      double rho = meas_package.raw_measurements_(0);
      double phi = meas_package.raw_measurements_(1);
      double rhodot = meas_package.raw_measurements_(2);

      double x = rho * cos(phi);
      double y = rho * sin(phi);

      double vx = rhodot * cos(phi);
      double vy = rhodot * sin(phi);
      double v = sqrt(vx * vx + vy * vy);

      x_ << x, y, v, rho, rhodot;
      
      //state covariance matrix
      //***** values can be tuned *****
      double std_radr2 = std_radr_*std_radr_;
      P_ << std_radr2, 0, 0, 0, 0,
            0, std_radr2, 0, 0, 0,
            0, 0, std_radrd_*std_radrd_, 0, 0,
            0, 0, 0, std_radphi_, 0,
            0, 0, 0, 0, std_radphi_;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) 
    {
      // Initialize state.
      // ***** Last three values below can be tuned *****
      x_ << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), 0, 0, 0.0;
      
      //state covariance matrix
      //***** values can be tuned *****
      P_ << std_laspx_*std_laspx_, 0, 0, 0, 0,
            0, std_laspy_*std_laspy_, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;
    }
    
    is_initialized_ = true;
    time_us_ = meas_package.timestamp_;
    return;
  }
  
  // Calculate delta t
  auto temp = meas_package.timestamp_;
  double delta_t = (temp - time_us_) / 1e6;

  // Update timestamp
  time_us_ = temp;

  // Predict
  Prediction(delta_t);

  // Measurement updates
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) 
    UpdateRadar(meas_package);
  else
    UpdateLidar(meas_package);
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
  lambda_ = 3 - n_x_;
  
  // Sigma point matrix
  MatrixXd xSigMat = MatrixXd(n_x_, 2*n_x_ + 1);
  
  // Computing the square root of P
  MatrixXd A = P_.llt().matrixL();
  
  xSigMat.col(0) = x_;
  for(int i = 0; i < n_x_; i++)
  {
    xSigMat.col(i+1) = x_ + std::sqrt(lambda_ + n_x_) * A.col(i);
    xSigMat.col(i+1 + n_x_) = x_ - std::sqrt(lambda_ + n_x_) * A.col(i);
  }
  
  lambda_ = 3 - n_aug_;
  
  // Augmented state
  VectorXd xAug = VectorXd(7);
  MatrixXd PAug = MatrixXd(7, 7);
  
  xAug.head(5) = x_;
  xAug(5) = 0;
  xAug(6) = 0;
  
  // Sigma point matrix
  MatrixXd xSigMatAug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  
  // Augmented covariance matrix
  MatrixXd Q = MatrixXd(2,2);
  Q << std_a_*std_a_, 0,
       0,             std_yawdd_*std_yawdd_;
  PAug.fill(0.0);
  PAug.topLeftCorner(5, 5) = P_;
  PAug.bottomRightCorner(2, 2) = Q;
  
  // Square root
  MatrixXd AAug = PAug.llt().matrixL();
  
  // Augmented sigma points
  xSigMatAug.col(0) = xAug;
  for(int i = 0; i < n_aug_; i++) 
  {
    xSigMatAug.col(i+1) = xAug + std::sqrt(lambda_ + n_aug_) * AAug.col(i);
    xSigMatAug.col(i+1 + n_aug_) = xAug - std::sqrt(lambda_ + n_aug_) * AAug.col(i);
  }
  
  // Sigma points prediction
  VectorXd v1 = VectorXd(5), v2 = VectorXd(5);
  
  for(int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    VectorXd col_i = xSigMatAug.col(i);
    double px = col_i(0);
    double py = col_i(1);
    double v = col_i(2);
    double yaw = col_i(3);
    double yawd = col_i(4);
    double v_aug = col_i(5);
    double v_yawdd = col_i(6);
    
    VectorXd original = col_i.head(5);
    
    // Avoid zero division
    if(yawd >= 1e-3)
    {
      v1 << (v / yawd) * (sin(yaw + yawd * delta_t) - sin(yaw)),
              (v / yawd) * (-cos(yaw + yawd * delta_t) + cos(yaw)),
              0,
              yawd * delta_t,
              0;
    } 
    else
    {
      v1 << v * cos(yaw) * delta_t,
              v * sin(yaw) * delta_t,
              0,
              yawd * delta_t,
              0;
    }
    
    // This portion stays the same
    v2 << 0.5 * v_aug * cos(yaw) * delta_t * delta_t,
            0.5 * v_aug * sin(yaw) * delta_t * delta_t,
            v_aug * delta_t,
            0.5 * v_yawdd * delta_t * delta_t,
            v_yawdd * delta_t;
    
    Xsig_pred_.col(i) << original + v1 + v2;
  }
  
  // Predicted state
  VectorXd xPredicted = VectorXd(n_x_);
  MatrixXd PPredicted = MatrixXd(n_x_, n_x_);
  
  xPredicted.fill(0.0);
  PPredicted.fill(0.0);
  
  for(int i = 0; i < 2 * n_aug_ + 1; i++) 
  {
    //set weights
    if (!i)
      weights_(i) = lambda_ / (lambda_ + n_aug_);
    else
      weights_(i) = 0.5 / (lambda_ + n_aug_);
    
    //predict state mean
    xPredicted += weights_(i) * Xsig_pred_.col(i);
  }
  
  for (int i = 0; i < 2 * n_aug_ + 1; i++) 
  {
    VectorXd xDiff = Xsig_pred_.col(i) - xPredicted;
    
    // Normalize angle
    while (xDiff(3) > M_PI)
      xDiff(3) -= 2 * M_PI;
    while (xDiff(3) <= -M_PI)
      xDiff(3) += 2*M_PI;
    // xDiff(3) = (xDiff(3) + M_PI) % 2*M_PI - M_PI;
    
    PPredicted += weights_(i) * xDiff * xDiff.transpose();
  }
  
  x_ = xPredicted;
  P_ = PPredicted;
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  int n_z = 2;
  
  // Sigma points matrix
  MatrixXd zSigMat = MatrixXd(n_z, 2 * n_aug_ + 1);
  
  // Predicted measurement
  VectorXd zPredicted = VectorXd(n_z);
  
  // Measurement covariance matrix
  MatrixXd S = MatrixXd(n_z,n_z);
  
  zSigMat.fill(0.0);
  zPredicted.fill(0.0);
  S.fill(0.0);
  
  for (int i = 0; i < 2 * n_aug_ + 1; i++) 
  {
    // Transform sigma points into measurement space
    VectorXd col_i = Xsig_pred_.col(i);
    double px = col_i(0);
    double py = col_i(1);
    
    zSigMat.col(i) << px,
                      py;
    
    zPredicted += weights_(i) * zSigMat.col(i);
  }
  
  for (int i = 0; i < 2 * n_aug_ + 1; i++) 
  {
    VectorXd zDiff = zSigMat.col(i) - zPredicted;
    S += weights_(i) * zDiff * zDiff.transpose();
  }
  
  MatrixXd R = MatrixXd(2,2);
  R << std_laspx_*std_laspx_, 0,
       0,                     std_laspy_*std_laspy_;
  S += R;
  
  // Radar measurement
  VectorXd z = VectorXd(n_z);
  
  z << meas_package.raw_measurements_(0),
       meas_package.raw_measurements_(1);
  
  // Cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  
  for (int i = 0; i < 2 * n_aug_ + 1; i++) 
  {
    VectorXd xDiff = Xsig_pred_.col(i) - x_;
    
    // Normalize angles
    // xDiff(3) = (xDiff(3) + M_PI) % 2*M_PI - M_PI;
    while (xDiff(3) > M_PI)
      xDiff(3) -= 2 * M_PI;
    while (xDiff(3) <= -M_PI)
      xDiff(3) += 2*M_PI;
    
    VectorXd zDiff = zSigMat.col(i) - zPredicted;

    Tc += weights_(i) * xDiff * zDiff.transpose();

  }
  
  // Residuals
  VectorXd zDiff = z - zPredicted;
  
  // NIS
  NIS_laser_ = zDiff.transpose() * S.inverse() * zDiff;
  
  // Kalman gain
  MatrixXd K = Tc * S.inverse();
  
  // State update
  x_ += K * zDiff;
  P_ -= K * S * K.transpose();
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  int n_z = 3;
  
  // Sigma points matrix
  MatrixXd zSigMat = MatrixXd(n_z, 2 * n_aug_ + 1);
  
  // Predicted measurement
  VectorXd zPredicted = VectorXd(n_z);
  
  // Measurement covariance
  MatrixXd S = MatrixXd(n_z,n_z);
  
  zSigMat.fill(0.0);
  zPredicted.fill(0.0);
  S.fill(0.0);

  double rho = 0, phi = 0, rho_d = 0;
  
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    VectorXd col_i = Xsig_pred_.col(i);
    double px = col_i(0);
    double py = col_i(1);
    double v = col_i(2);
    double yaw = col_i(3);
    double yaw_d = col_i(4);
    
    rho = sqrt(px*px + py*py);
    phi = atan2(py, px);
    rho_d = (px * v * cos(yaw) + py * v * sin(yaw)) / rho;
    
    zSigMat.col(i) << rho,
                      phi,
                      rho_d;
    
    zPredicted += weights_(i) * zSigMat.col(i);
  }
  
  for (int i = 0; i < 2 * n_aug_ + 1; i++) 
  {
    VectorXd zDiff = zSigMat.col(i) - zPredicted;
    // zDiff(1) = (zDiff(1) + M_PI) % 2*M_PI - M_PI; // Normalize angle
    while (zDiff(1) > M_PI)
      zDiff(1) -= 2 * M_PI;
    while (zDiff(1) <= -M_PI)
      zDiff(1) += 2*M_PI;
    
    S += weights_(i) * zDiff * zDiff.transpose();
  }
  
  // Add R to S
  MatrixXd R = MatrixXd(3, 3);
  R << std_radr_*std_radr_, 0, 0,
       0, std_radphi_*std_radphi_, 0,
       0, 0, std_radrd_*std_radrd_;
  S += R;
  
  VectorXd z = VectorXd(n_z);
  
  z << meas_package.raw_measurements_(0),
       meas_package.raw_measurements_(1),
       meas_package.raw_measurements_(2);
  
  // Cross correlation matrix Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  
  for (int i = 0; i < 2 * n_aug_ + 1; i++) 
  {
    VectorXd xDiff = Xsig_pred_.col(i) - x_;
    // xDiff(3) = (xDiff(3) + M_PI) % 2*M_PI - M_PI; // Normalize angle
    while (xDiff(3) > M_PI)
      xDiff(3) -= 2 * M_PI;
    while (xDiff(3) <= -M_PI)
      xDiff(3) += 2*M_PI;
    
    VectorXd zDiff = zSigMat.col(i) - zPredicted;
    // zDiff(1) = (zDiff(1) + M_PI) % 2*M_PI - M_PI; // Normalize angle
    while (zDiff(1) > M_PI)
      zDiff(1) -= 2 * M_PI;
    while (zDiff(1) <= -M_PI)
      zDiff(1) += 2*M_PI;
  
    Tc += weights_(i) * xDiff * zDiff.transpose();
  }
  
  // Residuals
  VectorXd zDiff = z - zPredicted;
  // zDiff(1) = (zDiff(1) + M_PI) % 2*M_PI - M_PI; // Normalize angle
  while (zDiff(1) > M_PI)
      zDiff(1) -= 2 * M_PI;
  while (zDiff(1) <= -M_PI)
      zDiff(1) += 2*M_PI;
  
  // NIS
  NIS_radar_ = zDiff.transpose() * S.inverse() * zDiff;
  
  // Kalman gain
  MatrixXd K = Tc * S.inverse();
  
  // State update
  x_ += K * zDiff;
  P_ -= K * S * K.transpose();
}