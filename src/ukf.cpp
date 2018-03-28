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
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 2.0;
  
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
  
  /**

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  // initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_ = false;
  
  // initialize time stamp when the state is true, in us
  time_us_ = 0;
  
  // set State dimension
  n_x_ = 5;
  
  //set Augmented state dimension by adding the process noise elements value
 n_aug_ = 7;
  
  // set Sigma point spreading parameter
  lambda_ = 3 - n_aug_;  // Rule of thump
  
  // Initialize predicted sigma points matrix, create matrix with predicted sigma points as columns
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1); // 5 X 15
  
  // Initialize vector for weights
  weights_ = VectorXd(2 * n_aug_ + 1); // 15

  
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /*****************************************************************************
   *  Initialization (1st measurment)
   ****************************************************************************/
	if (!is_initialized_) {
		/**
		  * Initialize the state x_ with the first measurement.
		  * Create the covariance matrix.
		  * Remember: you'll need to convert radar from polar to cartesian coordinates.
		*/

		
			// Initialize state measurement
			x_ << 1, 1, 5.199937e+00, 1.036644e-03, 2.072960e-02;// using the Lidar 1st measurment for initialization

			//  initialize state covariance matrix
			P_ << 0.15, 0, 0, 0, 0,
                            0, 0.15, 0, 0, 0,
                            0, 0, 10, 0, 0,
                            0, 0, 0, 1, 0,
                            0, 0, 0, 0, 10; // standard deviation of the lidar x and y measurements is 0.15

			// initialize timestamp
			time_us_ = meas_package.timestamp_;

			if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {

				/**
				Convert radar from polar to cartesian coordinates and initialize state [px,py,v,yaw,yaw_dot].
				*/
				/*
				float rho = meas_package.raw_measurements_[0];
				float theta = meas_package.raw_measurements_[1];
				float rho_dot = meas_package.raw_measurements_[2];
				float px = rho * cos(theta);
				float py = rho * sin(theta);
				float v = rho_dot;
				float yaw = theta;
				float yaw_dot = 0; // since we are having CTRV, so Turning rate is contstant.

				//set the state with the initial location and  velocity, yaw and yaw_dot
				x_ << px, py, v, yaw, yaw_dot;
							*/

				float ro = meas_package.raw_measurements_(0);
				float phi = meas_package.raw_measurements_(1);
				float ro_dot = meas_package.raw_measurements_(2);
				x_(0) = ro     * cos(phi);
				x_(1) = ro     * sin(phi);
				cout << " Initialization is done with Radar data!" << endl;




			}
			else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
				/**
				Initialize state.
				*/
				//set the state with the initial location (px,py) and zero velocity
				//x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0.0, 0.0, 0.0;
				x_(0) = meas_package.raw_measurements_[0];
				x_(1) = meas_package.raw_measurements_[1];

				cout << " Initialization is done with Lidar data!" << endl;

			}

			// done initializing
			is_initialized_ = true;
			return;
		}

		//compute the time elapsed between the current and previous measurements
		double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds
		time_us_ = meas_package.timestamp_;

		//Predict
		Prediction(dt);

		//Update 
		if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
			UpdateLidar(meas_package);
		}
		else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
			UpdateRadar(meas_package);
		}


	
  
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double dt) {
  /**
  This function estimates the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  
     /*****************************************************************************
   *  Generate Sigma points
   ****************************************************************************/
  
  //create augmented mean vector
  VectorXd x_aug_ = VectorXd(n_aug_);
  x_aug_.fill(0.0);

  //create augmented state covariance
  MatrixXd P_aug_ = MatrixXd(n_aug_, n_aug_);
  P_aug_.fill(0.0);	
  
  //create Augmented sigma points matrix
  MatrixXd Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug_.fill(0.0);	
  
  //create augmented mean state	
  x_aug_.head(5) = x_;
  x_aug_(5) = 0;
  x_aug_(6) = 0;

  //create augmented covariance matrix
  P_aug_.topLeftCorner(5,5) = P_;
  P_aug_(5,5) = std_a_*std_a_;
  P_aug_(6,6) = std_yawdd_*std_yawdd_;
  
  //create square root matrix
  MatrixXd L = P_aug_.llt().matrixL();
  
  //create augmented sigma points	
  Xsig_aug_.col(0)  = x_aug_;
  for (int i = 0; i< n_aug_; i++) {
    Xsig_aug_.col(i+1)       = x_aug_ + sqrt(lambda_ +n_aug_) * L.col(i);
    Xsig_aug_.col(i+1+n_aug_) = x_aug_ - sqrt(lambda_ +n_aug_) * L.col(i);
  }
  
     /*****************************************************************************
   *  Predict Sigma points
   ****************************************************************************/
  for (int i = 0; i< 2*n_aug_+1; i++) {
  
    //extract values for better readability
    double p_x = Xsig_aug_(0,i);
    double p_y = Xsig_aug_(1,i);
    double v = Xsig_aug_(2,i);
    double yaw = Xsig_aug_(3,i);
    double yawd = Xsig_aug_(4,i);
    double nu_a = Xsig_aug_(5,i);
    double nu_yawdd = Xsig_aug_(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*dt) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*dt) );
    }
    else {
        px_p = p_x + v*dt*cos(yaw);
        py_p = p_y + v*dt*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*dt;
    double yawd_p = yawd;
    
    //add noise
    px_p = px_p + 0.5*std_a_*dt*dt * cos(yaw);
    py_p = py_p + 0.5*std_a_*dt*dt * sin(yaw);
    v_p = v_p + std_a_*dt;

    yaw_p = yaw_p + 0.5*std_yawdd_*dt*dt;
    yawd_p = yawd_p + std_yawdd_*dt;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  
   }
 
     /*****************************************************************************
   *  Predict Mean and Covariance
   ****************************************************************************/
 
  // set weights
  double weight_0 = lambda_ /(lambda_ +n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights
    double weight = 0.5/(n_aug_+ lambda_);
    weights_(i) = weight;
  }
  
  //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ = x_+ weights_(i) * Xsig_pred_.col(i);
  }
  
  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }
  
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
   
   /*****************************************************************************
   *  Predict Lidar measurment (Convert from 5-dim state to 2- dim state)
   ****************************************************************************/
	cout << "Update Lidar!!" << endl;
  //set measurement dimension, Lidarr can measure distances px & py.
  int n_z_ = 2;
  
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_);
  z_pred.fill(0.0);	
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_, n_z_);
  S.fill(0.0);	
  
  //transform sigma points into measurement space
   for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
   
    // measurement model
    Zsig(0,i) = p_x;                   //px
    Zsig(1,i) = p_y;                   //py
    
  }
   
  //calculate mean predicted measurement
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
  cout << "" << endl;
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
 
  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z_, n_z_);
  R << std_laspx_*std_laspx_, 0,
	  0, std_laspy_*std_laspy_;
          
  S = S + R;
 
   /*****************************************************************************
   *  Update Lidar measurment
   ****************************************************************************/
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);
  
  //calculate cross correlation matrix
  Tc.fill(0.0);
  
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
	
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  
  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;
 
   //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
  
  // print the output
  cout << "x_ Lidar= " << x_ << endl;
  cout << "P_ Lidar= " << P_ << endl;
  
     /*****************************************************************************
   *  Calculate Lidar NIS
   ****************************************************************************/
  double NIS = z_diff.transpose() * S.inverse() * z_diff;
  cout << "NIS_Lidar= " << NIS << endl;

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
   /*****************************************************************************
   *  Predict Radar measurment( Convert from 5-dim state to 3-dim state)
   ****************************************************************************/
	cout << "Update Radar!!" << endl;

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z_ = 3;
  
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_, n_z_);
  
  //transform sigma points into measurement space
   for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //rho
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //rho_dot, TODO: securing division by zero needed to be added
  }
  //calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
  
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z_, n_z_);
  R <<    std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0,std_radrd_*std_radrd_;
  S = S + R;
   /*****************************************************************************
   *  Update Radar measurment
   ****************************************************************************/
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);
  
  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;
  
   //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  // print the output
  cout << "x_ Radar= " << x_ << endl;
  cout << "P_ Radar= " << P_ << endl;
  
     /*****************************************************************************
   *  Calculate Radar NIS
   ****************************************************************************/
  double NIS = z_diff.transpose() * S.inverse() * z_diff;
  cout << "NIS_Radar= " << NIS << endl;
  
}
