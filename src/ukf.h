#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
private:
 
  MatrixXd GenerateSigmaPoints(VectorXd x, MatrixXd P, double lambda, int n_sig);

  
  MatrixXd PredictSigmaPoints(MatrixXd Xsig, double delta_t, int n_x, int n_sig, double nu_am, double nu_yawdd);

 
  void NormalizeAngleOnComponent(VectorXd vector, int index);
public:

  // initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  // if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  // if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  // state vector
  VectorXd x_;

  // state covariance matrix
  MatrixXd P_;

  // predicted sigma points matrix
  MatrixXd Xsig_pred_;

  // time when the state is true
  long long time_us_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  // Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  // Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  // Radar measurement noise standard deviation radius in m
  double std_radr_;

  // Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  // Weights of sigma points
  VectorXd weights_;

  // State dimension
  int n_x_;

  // Augmented state dimension
  int n_aug_;

  // Sigma point spreading parameter
  double lambda_;

  // Sigma points dimension
  int n_sig_;

  // Radar measurement noise covariance matrix
  MatrixXd R_radar_;

  // Lidar measurement noise covariance matrix
  MatrixXd R_lidar_;

  // the current NIS for radar
  double NIS_radar_;

  // the current NIS for laser
  double NIS_laser_;


  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  void ProcessMeasurement(MeasurementPackage meas_package);


  void Prediction(double delta_t);

  
  void UpdateLidar(MeasurementPackage meas_package);

  
  void UpdateRadar(MeasurementPackage meas_package);
};

#endif /* UKF_H */
