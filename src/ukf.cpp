#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {

  // Mark this UKF instance as not initialized, initialization will be done with the
  // first incoming measurement.
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // Set state dimension
  n_x_ = 5;

  // Set augmented dimension
  n_aug_ = 7;

  // Refine spreading parameter
  lambda_ = 3 - n_aug_;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // Create the augmented sigma point matrix
  Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  // Create matrix with predicted sigma points as columns
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2.4;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  // Initialize the sigma weights
  sigma_weights_ = VectorXd(2*n_aug_+1);

  sigma_weights_(0) = lambda_/(lambda_+n_aug_);
  for (int i = 1; i < 2*n_aug_+1; ++i) {
    sigma_weights_(i) = 0.5/(lambda_+n_aug_);
  }

  //transform sigma points into measurement space
  int radar_n_z = 3;
  radar_Zsig_ = MatrixXd(radar_n_z, 2 * n_aug_ + 1);

  //transform sigma points into measurement space
  int lidar_n_z = 2;
  lidar_Zsig_ = MatrixXd(lidar_n_z, 2 * n_aug_ + 1);

  radar_S_ = MatrixXd(3,3);
  lidar_S_ = MatrixXd(2,2);
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  // Ignore measurement types that are not enabled.
  if (!use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      return;
  }
  if (!use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER) {
      return;
  }

  /*
   * Unscented Kalman Filter Algorithm
   *
   * Handle Initial Measurement and Sanity Check
   *
   * Prediction:
   * - Generate Augmented Sigma Points
   * - Predict Sigma Points
   * - Predict Mean and Covariance
   *
   * Update:
   * - Predict Measurement
   * - Update State
   */

  if (!is_initialized_ || !SanityCheck(meas_package)) {
      // Processing of the initial measurement, or reset back to
      // initialization if there is an issue with the measurement.
      x_.fill(0.0);
      if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        double ro = meas_package.raw_measurements_[0];
        double phi = meas_package.raw_measurements_[1];
        x_(0) = ro*cos(phi);
        x_(1) = ro*sin(phi);
      } else {
        x_(0) = meas_package.raw_measurements_[0];
        x_(1) = meas_package.raw_measurements_[1];
      }
      time_us_ = meas_package.timestamp_;
      is_initialized_ = true;

      return;
  }

  double delta_t = (meas_package.timestamp_ - time_us_)/1000000.0; // Convert us to seconds
  time_us_ = meas_package.timestamp_; // Record last measurement timestamp


  Prediction(delta_t);

  /* The Update step depends on the type of measurement */
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(meas_package);
  } else {
    UpdateLidar(meas_package);
  }
}

/**
 * Checks if the incoming measurement is sane or is so out of spec that it is likely to cause
 * an issue for the algorithm.
 * @param {MeasurementPackage} meas_package
 */
bool UKF::SanityCheck(MeasurementPackage meas_package) {
  // Rule #1 Check that the timestamp is increasing
  if (time_us_ >= meas_package.timestamp_) {
      // FIXME Handle roll-over.
      return false;
  }

  // Rule #2 Check that the delta_t is a valid range (< 1 second)
  double delta_t = (meas_package.timestamp_ - time_us_)/1000000.0;
  if (delta_t > 1.0) {
      return false;
  }

  return true;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /*
   * Prediction:
   * - Generate Augmented Sigma Points
   * - Predict Sigma Points
   * - Predict Mean and Covariance
   */

  GenerateAugmentedSigmaPoints();
  SigmaPointPrediction(delta_t);
  PredictMeanAndCovariance();
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /*
   * Lidar Update:
   * - Predict Measurement
   * - Update State Mean and Covariance
   */

  PredictLidarMeasurement();

  double px = meas_package.raw_measurements_[0];
  double py = meas_package.raw_measurements_[1];

  UpdateStateMeanAndCovarianceUsingLidar(px, py);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /*
   * Radar Update:
   * - Predict Measurement
   * - Update State Mean and Covariance
   */

  PredictRadarMeasurement();

  double rho = meas_package.raw_measurements_[0];
  double phi = meas_package.raw_measurements_[1];
  double rho_dot = meas_package.raw_measurements_[2];

  UpdateStateMeanAndCovarianceUsingRadar(rho, phi, rho_dot);
}

void UKF::GenerateAugmentedSigmaPoints() {

  // Create the augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  // Create an augmented mean state
  x_aug.head(n_x_) = x_;
  x_aug(n_x_) = 0;   /* mean is zero */
  x_aug(n_x_+1) = 0; /* mean is zero */

  // Create the augmented state covariance matrix
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  // Fill in the augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_*std_a_;
  P_aug(n_x_+1, n_x_+1) = std_yawdd_*std_yawdd_;

  // Compute the square root of the augmented covariance matrix
  MatrixXd A = P_aug.llt().matrixL();

  // Generate the augmented sigma points
  Xsig_aug_.col(0) = x_aug;
  float f = sqrt(lambda_+n_aug_);
  for (int i = 0; i < n_aug_; ++i) {
    Xsig_aug_.col(i+1) = x_aug + A.col(i) * f;
    Xsig_aug_.col(i+n_aug_+1) = x_aug - A.col(i) * f;
  }
  
  // Updated: Xsig_aug_
}

void UKF::SigmaPointPrediction(double delta_t /* time diff is seconds */) {

  // Predict sigma points but make sure to avoid division by zero
  for (int i = 0; i < 2*n_aug_+1; ++i) {
    // Step 1
    Xsig_pred_.col(i) = Xsig_aug_.col(i).head(n_x_);
    // Step 2
    float v = Xsig_aug_(2,i);
    float phi = Xsig_aug_(3,i);
    float phi_dot = Xsig_aug_(4,i);
    float nu_v_dot = Xsig_aug_(5,i);
    float nu_phi_dotdot = Xsig_aug_(6,i);
    if (phi_dot == 0.0) {
      /* Avoid division by zero */
      Xsig_pred_(0,i) += v * cos(phi) * delta_t;
      Xsig_pred_(1,i) += v * sin(phi) * delta_t;
    } else {
      Xsig_pred_(0,i) += v / phi_dot * (sin(phi + phi_dot * delta_t) - sin(phi));
      Xsig_pred_(1,i) += v / phi_dot * (-cos(phi + phi_dot * delta_t) + cos(phi));
    }
    Xsig_pred_(3,i) += delta_t * phi_dot;
    // Step 3
    Xsig_pred_(0,i) += 0.5 * delta_t * delta_t * cos(phi) * nu_v_dot;
    Xsig_pred_(1,i) += 0.5 * delta_t * delta_t * sin(phi) * nu_v_dot;
    Xsig_pred_(2,i) += delta_t * nu_v_dot;
    Xsig_pred_(3,i) += 0.5 * delta_t * delta_t * nu_phi_dotdot;
    Xsig_pred_(4,i) += delta_t * nu_phi_dotdot;
  }

  // Updated: Xsig_pred_
}

void UKF::PredictMeanAndCovariance() {

  // Predict state mean
  x_ = Xsig_pred_ * sigma_weights_;

  // Predict state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; ++i) {
    VectorXd diff = (Xsig_pred_.col(i) - x_);
    //angle normalization
    while (diff(1)> M_PI) diff(1)-=2.*M_PI;
    while (diff(1)<-M_PI) diff(1)+=2.*M_PI;
    P_ += (diff*diff.transpose())*sigma_weights_(i);
  }

  // Updated: x_ and P_
}

void UKF::PredictRadarMeasurement() {

  // Set measurement dimension, radar can measure r, phi, and r_dot
  int radar_n_z = 3;

  for (int i = 0; i < 2*n_aug_+1; ++i) {
    float px, py, v, phi;
    px  = Xsig_pred_(0,i);
    py  = Xsig_pred_(1,i);
    v   = Xsig_pred_(2,i);
    phi = Xsig_pred_(3,i);
      
    radar_Zsig_(0,i) = sqrt(px*px + py*py);
    radar_Zsig_(1,i) = atan2(py, px);
    radar_Zsig_(2,i) = (px*cos(phi)*v + py*sin(phi)*v)/sqrt(px*px + py*py);
  }

  // Calculate mean predicted measurement
  radar_z_pred_ = radar_Zsig_ * sigma_weights_;

  // Calculate innovation covariance matrix S
  radar_S_.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; ++i) {
    VectorXd diff = VectorXd(radar_n_z);
    diff = radar_Zsig_.col(i) - radar_z_pred_;
    //angle normalization
    while (diff(1)> M_PI) diff(1)-=2.*M_PI;
    while (diff(1)<-M_PI) diff(1)+=2.*M_PI;
    radar_S_ += (diff * diff.transpose()) * sigma_weights_(i);
  }
  radar_S_(0,0) += std_radr_*std_radr_;
  radar_S_(1,1) += std_radphi_*std_radphi_;
  radar_S_(2,2) += std_radrd_*std_radrd_;

  // Updated radar_z_pred_ and radar_S_
}

void UKF::UpdateStateMeanAndCovarianceUsingRadar(double rho, double phi, double rho_dot) {

  // Set measurement dimension, radar can measure r, phi, and r_dot
  int radar_n_z = 3;

  // Create a vector from the incoming radar measurement
  VectorXd z = VectorXd(radar_n_z);
  z <<
    rho,     // rho in m
    phi,     // phi in rad
    rho_dot; // rho_dot in m/s

  // Calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, radar_n_z);

  Tc.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; ++i) {
    VectorXd xdiff = (Xsig_pred_.col(i) - x_);
    // Angle normalization
    while (xdiff(1)> M_PI) xdiff(1)-=2.*M_PI;
    while (xdiff(1)<-M_PI) xdiff(1)+=2.*M_PI;
    VectorXd zdiff = (radar_Zsig_.col(i) - radar_z_pred_);
    // Angle normalization
    while (zdiff(1)> M_PI) zdiff(1)-=2.*M_PI;
    while (zdiff(1)<-M_PI) zdiff(1)+=2.*M_PI;
    Tc += xdiff * zdiff.transpose() * sigma_weights_(i);
  }

  // Calculate Kalman gain K
  MatrixXd K = Tc * radar_S_.inverse();

  // Update state mean and covariance matrix
  x_ = x_ + K * (z - radar_z_pred_);
  P_ = P_ - K * radar_S_ * K.transpose();

  // Updated: x_ and P_
}

void UKF::PredictLidarMeasurement() {

  // Set measurement dimension, lidar can measure px and py
  int lidar_n_z = 2;

  for (int i = 0; i < 2*n_aug_+1; ++i) {
    float px, py, v, phi;
    px  = Xsig_pred_(0,i);
    py  = Xsig_pred_(1,i);
    v   = Xsig_pred_(2,i);
    phi = Xsig_pred_(3,i);

    lidar_Zsig_(0,i) = px;
    lidar_Zsig_(1,i) = py;
  }

  // Calculate mean predicted measurement
  lidar_z_pred_ = lidar_Zsig_ * sigma_weights_;

  // Calculate innovation covariance matrix S
  lidar_S_.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; ++i) {
    VectorXd diff = VectorXd(lidar_n_z);
    diff = lidar_Zsig_.col(i) - lidar_z_pred_;
    //angle normalization
    while (diff(1)> M_PI) diff(1)-=2.*M_PI;
    while (diff(1)<-M_PI) diff(1)+=2.*M_PI;
    lidar_S_ += (diff * diff.transpose()) * sigma_weights_(i);
  }
  lidar_S_(0,0) += std_laspx_*std_laspx_;
  lidar_S_(1,1) += std_laspy_*std_laspy_;

  // Updated: lidar_z_pred_ and lidar_S_
}

void UKF::UpdateStateMeanAndCovarianceUsingLidar(double px, double py) {

  // Set measurement dimension, lidar can measure px and py
  int lidar_n_z = 2;

  // Create a vector from the incoming radar measurement
  VectorXd z = VectorXd(lidar_n_z);
  z <<
    px,     // px in m
    py;     // py in m

  // Calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, lidar_n_z);

  Tc.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; ++i) {
    VectorXd xdiff = (Xsig_pred_.col(i) - x_);
    //angle normalization
    while (xdiff(1)> M_PI) xdiff(1)-=2.*M_PI;
    while (xdiff(1)<-M_PI) xdiff(1)+=2.*M_PI;
    VectorXd zdiff = (lidar_Zsig_.col(i) - lidar_z_pred_);
    //angle normalization
    while (zdiff(1)> M_PI) zdiff(1)-=2.*M_PI;
    while (zdiff(1)<-M_PI) zdiff(1)+=2.*M_PI;
    Tc += xdiff * zdiff.transpose() * sigma_weights_(i);
  }

  // Calculate Kalman gain K
  MatrixXd K = Tc * lidar_S_.inverse();

  // Update state mean and covariance matrix
  x_ = x_ + K * (z - lidar_z_pred_);
  P_ = P_ - K * lidar_S_ * K.transpose();

  // Updated: x_ and P_
}
