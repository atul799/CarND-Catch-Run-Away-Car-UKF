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
	//use_laser_ = false;
	// if this is false, radar measurements will be ignored (except during init)
	use_radar_ = true;
	//use_radar_ = false;

	// initial state vector
	//5 state vectors are px/py/v/yaw/yawrate
	x_ = VectorXd(5);
	x_.fill(0);

	// initial covariance matrix
	P_ = MatrixXd(5, 5);
	P_.fill(0);

	// Process noise standard deviation longitudinal acceleration in m/s^2
	//std_a_ = 30;
	//std_a_ = 0.5;
	//std_a_ = 1.0;
	//std_a_ = 0.259; //reference for bicycle acceleration is taken from study paper:https://www.diva-portal.org/smash/get/diva2:795377/FULLTEXT01.pdf
	std_a_ = 0.5;
	//for runaway car
	//std_a_ = 3;
	//std_a_ = 0.1;
	// Process noise standard deviation yaw acceleration in rad/s^2
	//std_yawdd_ = 30;
	//std_yawdd_ = 1;
	std_yawdd_ = 0.5;
	//for runaway car
	//std_yawdd_ = 0.1;

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

	//ukf object initialization state, initially set to false, set to true in first call of ProcessMeasurement
	is_initialized_=false;

	// State dimension
	n_x_=5;

	//Augmented state dimension
	n_aug_=n_x_+2;

	//sigma point length
	n_sig_=2*n_aug_+1;

	//predicted sigma points matrix
	Xsig_pred_=MatrixXd(n_x_,n_sig_);
	Xsig_pred_.fill(0);
	//time when the state is true, in us
	time_us_=0;


	//Sigma point spreading parameter
	lambda_=3-n_aug_;


	///* Weights of sigma points
	weights_=VectorXd(n_sig_);
	//value of weights_ is static, genece init in constructor
	//weights_.fill(0.0);
	weights_.fill(1/(2*(lambda_+n_aug_)));
	weights_(0)=lambda_/(lambda_+n_aug_);

	//NIS values init

	NIS_=0;

	//NIS of laser
	NIS_laser_=0;

	//NIS of radar
	NIS_radar_=0;

	//R_laser
	R_laser_=MatrixXd(2,2);
	R_laser_.fill(0);
	R_laser_(0,0)=std_laspx_*std_laspx_;
	R_laser_(1,1)=std_laspy_*std_laspy_;


	//R_radar
	R_radar_ = MatrixXd(3,3);
	R_radar_.fill(0.0);
	R_radar_(0,0) =std_radr_*std_radr_;
	R_radar_(1,1) =std_radphi_*std_radphi_;
	R_radar_(2,2) =	std_radrd_*std_radrd_;



}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

	/*****************************************************************************
	 *   Initialization
	 *****************************************************************************/
	if (!is_initialized_) {
		/**
		 * Initialize the state x_ with the first measurement.
		 * Create the covariance matrix.
		 * convert radar from polar to cartesian coordinates.
		 */
		//init cov matrix
		P_ <<   1, 0, 0, 0, 0,
				0, 1, 0, 0, 0,
				0, 0, 1, 0, 0,
				0, 0, 0, 1, 0,
				0, 0, 0, 0, 1;


		// first measurement
		//cout << "UKF: " << endl;
		//if sensor data is from RADAR, measurement data --> rho/phi and rhodot
		if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
			/**
			      Convert radar from polar to cartesian coordinates and initialize state.
			 */
			float rho=meas_package.raw_measurements_[0];
			float phi=meas_package.raw_measurements_[1];
			float rhodot=meas_package.raw_measurements_[2];

			//convert polar to cartesian cords
			//rho,phi is  specified anticlockwise from x (considered vertical) and y considered
			//horiz pointing to left
			float px= rho*cos(phi);
			float py=rho*sin(phi);
			float vx=rhodot*cos(phi);
			float vy=rhodot*sin(phi);
			float v=sqrt(vx*vx+vy*vy); // rhodot :))
			cout <<"Radar init, rho,phi,px,py,vx,vy,rhodot,v"<<rho<<phi<<px<<py<<vx<<vy<<v<<rhodot<<endl;
			x_<<px,py,v,0,0;

			/*
			 * P_ <<   1, 0, 0, 0, 0,
					0, 1, 0, 0, 0,
					0, 0, 1, 0, 0,
					0, 0, 0, 1000, 0,
					0, 0, 0, 0, 1000; */
			//cout <<"Radar init, px,py,vx,vy"<<x_ <<endl;
		}
		//if sensor data is from LIDAR, measurement data --> px and py directly
		else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
			/**
			      Initialize state.
			 */
			//set the state with the initial location and zero velocity


			x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0,0;
			/* tried individual P_ for laser and radar init representing uncertainity in
			 *directly measured vs nonmeasured values.Assigned 1 for measured and 1000 for unmeasured(inferred by kalman)
			 * however, the cov matrix valuies blow up after a while, so initiated as 1
			 * P_ <<  1, 0, 0, 0, 0,
					0, 1, 0, 0, 0,
					0, 0, 1000, 0, 0,
					0, 0, 0, 1000, 0,
					0, 0, 0, 0, 1000; */

			cout <<"Lidar init:px,py,vx,vy" <<endl;
			cout <<x_ <<endl;
		}
		//save initial time stamp
		time_us_ = meas_package.timestamp_;
		// done initializing, no need to predict or update

		cout <<"Init Done"<<endl;
		is_initialized_ = true;
		return;
	}

	//calculate dt
	double dt = (meas_package.timestamp_ - time_us_) / 1000000.0; //dt - expressed in seconds

	time_us_ = meas_package.timestamp_;

	/***********************************************************
	 * Use the sensor type to perform the Predict/update step.
	 ***********************************************************/

	if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
		//cout << "RADAR data is being used "<<endl;
		//predict only if radar data use_radar_ is true
		Prediction(dt);
		//cout <<"Pred for radar done"<<endl;
		// Radar updates
		UpdateRadar(meas_package);
		//cout <<"update for radar done"<<endl;

	} else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_){
		//cout << "LASER meas is being used  "<<endl;
		//predict only if laser data use_laser_ is true
		Prediction(dt);
		//cout <<"Pred for Laser done"<<endl;
		// Laser updates
		//cout <<"update start laser "<<endl;
		UpdateLidar(meas_package);
		//UpdateLidar_unscented(meas_package);
		//cout <<"update for laser done"<<endl;
	}

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
	/**

  Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
	 */

	/**********************************
	 * Augmented Sigma point Generation
	 **********************************/
	//create augmented mean vector
	VectorXd x_aug = VectorXd(n_aug_);
	x_aug.fill(0);
	//create augmented state covariance
	MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
	//initialize all with .fill() method, is equivalent to MatrixXd::Zero(n_aug,n_aug)
	P_aug.fill(0.0);

	//create sigma point matrix
	MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);
	Xsig_aug.fill(0);

	//create augmented mean state
	x_aug.head(n_x_)=x_;

	//create augmented covariance matrix
	//NOTE to self::std_a and std_yawdd needs to be sqaured for covariance as they are given as std dev
	//and not as variance!!
	MatrixXd Q = MatrixXd(2, 2);
	Q <<    std_a_*std_a_,0,
			0,std_yawdd_*std_yawdd_;


	//add P to topleft corner
	P_aug.topLeftCorner(n_x_, n_x_)=P_;

	P_aug.bottomRightCorner(2,2)=Q;

	//create square root matrix

	MatrixXd A = P_aug.llt().matrixL();

	//create augmented sigma points
	Xsig_aug.col(0)=x_aug;

	for(int i=0;i<n_aug_;++i) {
		Xsig_aug.col(i+1)     = x_aug + sqrt(lambda_+n_aug_) * A.col(i);
		Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * A.col(i);

	}

	/****************************
	 * Sigma points prediction
	 ***************************/
	//predict sigma points

		for (int i = 0; i< n_sig_; i++)
		{
			//extract values for better readability
			double p_x = Xsig_aug(0,i);
			double p_y = Xsig_aug(1,i);
			double v = Xsig_aug(2,i);
			double yaw = Xsig_aug(3,i);
			double yawd = Xsig_aug(4,i);
			double nu_a = Xsig_aug(5,i);
			double nu_yawdd = Xsig_aug(6,i);

			//predicted state values
			double px_p, py_p;

			//avoid division by zero
			//ctrv model
			if (fabs(yawd) > 0.001) {
				px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
				py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
			}
			else {
				px_p = p_x + v*delta_t*cos(yaw);
				py_p = p_y + v*delta_t*sin(yaw);
			}

			double v_p = v;
			double yaw_p = yaw + yawd*delta_t;
			double yawd_p = yawd;

			//add noise
			px_p   += 0.5*nu_a*delta_t*delta_t * cos(yaw);
			py_p   += 0.5*nu_a*delta_t*delta_t * sin(yaw);
			v_p    += nu_a*delta_t;

			yaw_p  += 0.5*nu_yawdd*delta_t*delta_t;
			yawd_p += nu_yawdd*delta_t;

			//write predicted sigma point into right column
			Xsig_pred_(0,i) = px_p;
			Xsig_pred_(1,i) = py_p;
			Xsig_pred_(2,i) = v_p;
			Xsig_pred_(3,i) = yaw_p;
			Xsig_pred_(4,i) = yawd_p;
		}

	/**************************************************
	 * Predicted Mean and covariance
	 **************************************************/
	//predict state mean
	//weights=(15,1) Xsig_pred=5x15, x=Xsig_pred*weights=5x1
	x_.fill(0);
	x_=Xsig_pred_*weights_;

	//predict state covariance matrix
	//x=5x1, Xsig_pred=5x15
	//xk_x=5x15
	//weights=(15,1) ,

	//MatrixXd xk_x= MatrixXd(n_x_,n_sig_);
	//Xsig_pred_.colwise()-=x_; //5x15

	//MatrixXd P_Partial=xk_x*xk_x.transpose();  //5x5
	P_.fill(0.0);
	for(int i=0;i<n_sig_;i++) {
		// State difference
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		//Angle normalization
		while (x_diff(3) > M_PI)  x_diff(3)-=2*M_PI;
		while (x_diff(3) < -M_PI)  x_diff(3)+=2*M_PI;
		P_=P_ + weights_(i)*x_diff*x_diff.transpose();
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

	//calc predicted state, H, measurement function  is linear for Lidar (px,py measured directly)
	MatrixXd H_laser =MatrixXd(2,n_x_);
	H_laser << 1,0,0,0,0,
			0,1,0,0,0;
	VectorXd z_pred = H_laser * x_;
	VectorXd z=meas_package.raw_measurements_;
	//y is difference between predicted and measured state
	VectorXd y = z - z_pred;


	//aply kalman filter equations and update state and covariance mtx with kalman gain,K
	//the statements below are common to linear and extended kalman, can be created in a function,
	//function stack vs inline for processing speed....

	MatrixXd Ht = H_laser.transpose();
	MatrixXd S = H_laser * P_ * Ht + R_laser_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;
	//cout <<"nach R_laser"<<endl;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_laser) * P_;

	//cout<<"Updated x_ LASER: "<<x_<<endl;
	//cout<<"Updated P_ LASER:  "<<P_<<endl;


	NIS_laser_ = y.transpose() * S.inverse() * y;
	//cout<<"NIS_laser: "<<NIS_laser_<<endl;
	NIS_=NIS_laser_;
}


/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar_unscented(MeasurementPackage meas_package) {
	/**
	Use lidar data to update the belief about the object's
	  position. Modify the state vector, x_, and covariance, P_.

	  You'll also need to calculate the lidar NIS.
	 */
	//Unscented transform on laser measurement
	//set measurement dimension, radar can measure r, phi, and r_dot
	int n_z = 2;
	//create matrix for sigma points in measurement space
	MatrixXd Zsig = MatrixXd(n_z, n_sig_);
	Zsig.fill(0.0);
	Zsig=Xsig_pred_.topLeftCorner(n_z, n_sig_);
	//transform sigma points into measurement space

	/*for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points

		// extract values for better readibility
		double p_x = Xsig_pred_(0,i);
		double p_y = Xsig_pred_(1,i);
		double v  = Xsig_pred_(2,i);
		double yaw = Xsig_pred_(3,i);

		double v1 = cos(yaw)*v;
		double v2 = sin(yaw)*v;

		// measurement model
		double rho=sqrt(p_x*p_x + p_y*p_y);
		Zsig(0,i) =    rho ;                    //rho
		Zsig(1,i) = atan2(p_y,p_x);									//phi
		while (Zsig(1,i)> M_PI) Zsig(1,i)-=2.*M_PI;
		while (Zsig(1,i)<-M_PI) Zsig(1,i)+=2.*M_PI;
		if (fabs(rho) < 0.0001) {
			Zsig(2,i) =0.0;
		} else {
			Zsig(2,i) = (p_x*v1 + p_y*v2 ) / rho; //r_dot
		}

	} */

	//// measurement model
	//mean predicted measurement
	VectorXd z_pred = VectorXd(n_z);
	z_pred.fill(0.0);
	for (int i=0; i < n_sig_; i++) {
		z_pred = z_pred + weights_(i) * Zsig.col(i);
	}


	//innovation covariance matrix S
	MatrixXd S = MatrixXd(n_z,n_z);
	S.fill(0.0);
	for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points
		//residual
		VectorXd z_diff = Zsig.col(i) - z_pred;

		//angle normalization
		while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
		while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

		S = S + weights_(i) * z_diff * z_diff.transpose();
	}

	//Add noise from radar meas
	S = S + R_laser_;
	//create matrix for cross correlation Tc
	MatrixXd Tc = MatrixXd(n_x_, n_z);
	//calculate cross correlation matrix
	Tc.fill(0.0);
	for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points

		//residual
		VectorXd z_diff_c = Zsig.col(i) - z_pred;
		//angle normalization
		while (z_diff_c(1)> M_PI) z_diff_c(1)-=2.*M_PI;
		while (z_diff_c(1)<-M_PI) z_diff_c(1)+=2.*M_PI;

		// state difference
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		//angle normalization
		while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
		while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

		Tc = Tc + weights_(i) * x_diff * z_diff_c.transpose();
	}
	//Kalman gain K;
	MatrixXd K = Tc * S.inverse();

	VectorXd z=meas_package.raw_measurements_;

	//residual
	VectorXd z_diff = z - z_pred;
	//cout <<"z_diff"<<endl;
	//cout <<z_diff<<endl;

	//angle normalization
	while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
	while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

	//update state mean and covariance matrix
	x_ = x_ + K * z_diff;
	P_ = P_ - K*S*K.transpose();
	//cout<<"Updated x_ laser x: "<<x_<<endl;
	//cout<<"Updated P_ laser:  "<<P_<<endl;
	NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;
	//cout<<"NIS_laser: "<<NIS_laser_<<endl;
	NIS_=NIS_laser_;

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
	//set measurement dimension, radar can measure r, phi, and r_dot
	int n_z = 3;
	//create matrix for sigma points in measurement space
	MatrixXd Zsig = MatrixXd(n_z, n_sig_);
	Zsig.fill(0.0);
	//transform sigma points into measurement space
	for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points

		// extract values for better readibility
		double p_x = Xsig_pred_(0,i);
		double p_y = Xsig_pred_(1,i);
		double v  = Xsig_pred_(2,i);
		double yaw = Xsig_pred_(3,i);

		double v1 = cos(yaw)*v;
		double v2 = sin(yaw)*v;

		// measurement model
		double rho=sqrt(p_x*p_x + p_y*p_y);
		Zsig(0,i) =    rho ;                    //rho
		Zsig(1,i) = atan2(p_y,p_x);									//phi
		while (Zsig(1,i)> M_PI) Zsig(1,i)-=2.*M_PI;
		while (Zsig(1,i)<-M_PI) Zsig(1,i)+=2.*M_PI;
		if (fabs(rho) < 0.0001) {
			Zsig(2,i) =0.0;
		} else {
			Zsig(2,i) = (p_x*v1 + p_y*v2 ) / rho; //r_dot
		}

	}

	//// measurement model
	//mean predicted measurement
	VectorXd z_pred = VectorXd(n_z);
	z_pred.fill(0.0);
	for (int i=0; i < n_sig_; i++) {
		z_pred = z_pred + weights_(i) * Zsig.col(i);
	}


	//innovation covariance matrix S
	MatrixXd S = MatrixXd(n_z,n_z);
	S.fill(0.0);
	for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points
		//residual
		VectorXd z_diff = Zsig.col(i) - z_pred;

		//angle normalization
		while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
		while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

		S = S + weights_(i) * z_diff * z_diff.transpose();
	}

	//Add noise from radar meas
	S = S + R_radar_;
	//create matrix for cross correlation Tc
	MatrixXd Tc = MatrixXd(n_x_, n_z);
	//calculate cross correlation matrix
	Tc.fill(0.0);
	for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points

		//residual
		VectorXd z_diff_c = Zsig.col(i) - z_pred;
		//angle normalization
		while (z_diff_c(1)> M_PI) z_diff_c(1)-=2.*M_PI;
		while (z_diff_c(1)<-M_PI) z_diff_c(1)+=2.*M_PI;

		// state difference
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		//angle normalization
		while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
		while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

		Tc = Tc + weights_(i) * x_diff * z_diff_c.transpose();
	}
	//Kalman gain K;
	MatrixXd K = Tc * S.inverse();

	VectorXd z=meas_package.raw_measurements_;

	//residual
	VectorXd z_diff = z - z_pred;
	//cout <<"z_diff"<<endl;
	//cout <<z_diff<<endl;

	//angle normalization
	while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
	while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

	//update state mean and covariance matrix
	x_ = x_ + K * z_diff;
	P_ = P_ - K*S*K.transpose();
	//cout<<"Updated x_ RADAR x: "<<x_<<endl;
	//cout<<"Updated P_ RADAR:  "<<P_<<endl;
	NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
	//cout<<"NIS_radar: "<<NIS_radar_<<endl;
	NIS_=NIS_radar_;


}
