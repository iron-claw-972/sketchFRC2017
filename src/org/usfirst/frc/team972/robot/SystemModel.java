package org.usfirst.frc.team972.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SystemModel {

	//x is m, y is m, v is m/s, theta is degrees
	double x_k = 0.0;
	double y_k = 0.0;
	double v_xk = 0.0;
	double v_yk = 0.0;
	double a_xk = 0.0;
	double a_yk = 0.0;
	double theta_k = 0.0; // at t=k
	
	// The following variables are private to make it less confusing to use
	private double x_k1 = 0.0;
	private double y_k1 = 0.0;
	double v_xk1 = 0.0;
	double v_yk1 = 0.0;
	double a_xk1 = 0.0;
	double a_yk1 = 0.0;
	private double theta_k1 = 0.0; // at t=k+1

	/**
	 * Updates system model.
	 *
	 * @param gyro	Gyro angle
	 * @param dT	Loop time (change in time) in seconds
	 */
	public void updateSys(double gyro, double accel_x, double accel_y, double dT) {
		x_k1 = x_k + (v_xk * dT) + (a_xk * Math.pow(dT, 2) / 2);
		y_k1 = y_k + (v_yk * dT) + (a_yk * Math.pow(dT, 2) / 2);
		
		v_xk1 = v_xk + (a_xk * dT);
		v_yk1 = v_yk + (a_yk * dT);
		
		v_xk1 = (1 - Constants.SYSVEL) * Math.sin(theta_k * Math.PI / 180) * ((leftv_k + rightv_k) / 2) + Constants.SYSVEL * v_xk1;
		v_yk1 = (1 - Constants.SYSVEL) * Math.cos(theta_k * Math.PI / 180) * ((leftv_k + rightv_k) / 2) + Constants.SYSVEL * v_yk1;
		
		double accel_X_IMU =  (IMU.getAccelX() * Math.cos(theta_k * Math.PI / 180) + IMU.getAccelY() * Math.sin(theta_k * Math.PI / 180));
		double accel_Y_IMU =  (IMU.getAccelX() * Math.sin(theta_k * Math.PI / 180) + IMU.getAccelY() * Math.cos(theta_k * Math.PI / 180));
		
		a_xk1 = (1 - Constants.SYSACC) * Math.sin(theta_k * Math.PI / 180) * ((lefta_k + righta_k) / 2) + Constants.SYSACC * accel_X_IMU;
		a_xk1 = (1 - Constants.SYSACC) * Math.cos(theta_k * Math.PI / 180) * ((lefta_k + righta_k) / 2) + Constants.SYSACC * accel_Y_IMU;
		
		double angle_from_encoders = ((180 / (Math.PI * Constants.ROBOT_WIDTH)) * (leftx_k - rightx_k)) % 360.0;
		if (angle_from_encoders > 180) {
			angle_from_encoders = -360 + angle_from_encoders;
		} else if (angle_from_encoders < -180) {
			angle_from_encoders = 360 + angle_from_encoders;
		}
		double angle_from_gyro = gyro % 360.0;
		if (angle_from_gyro > 180) {
			angle_from_gyro = -360 + angle_from_gyro;
		} else if (angle_from_gyro < -180) {
			angle_from_gyro = 360 + angle_from_gyro;
		}
		if (angle_from_encoders < -90 && angle_from_gyro > 90) {
			theta_k1 = ((1 - Constants.PHI) * (360 + angle_from_encoders)) + (Constants.PHI * angle_from_gyro);
		} else if (angle_from_encoders > 90 && angle_from_gyro < -90) {
			theta_k1 = ((1 - Constants.PHI) * (-360 + angle_from_encoders)) + (Constants.PHI * angle_from_gyro);
		} else {
			theta_k1 = ((1 - Constants.PHI) * angle_from_encoders) + (Constants.PHI * angle_from_gyro);
		}
		if (theta_k1 > 180) {
			theta_k1 = -360 + theta_k1;
		} else if (theta_k1 < -180) {
			theta_k1 = 360 + theta_k1;
		}
		
		x_k = x_k1;
		y_k = y_k1;
		v_xk = v_xk1;
		v_yk = v_yk1;
		a_xk = a_xk1;
		a_yk = a_yk1;
		theta_k = theta_k1;
	}
	
	/**
	 * Set to a new state
	 * 
	 * @param x_state		New x position in meters
	 * @param y_state		New y position in meters
	 * @param v_state		New velocity in m/s
	 * @param theta_state	New angle in degrees
	 */
	public void setSysState(double x_state, double y_state, double vx_state, double vy_state, double theta_state) {
		x_k = x_state;
		y_k = y_state;
		v_xk = vx_state;
		v_yk = vy_state;
		theta_k = theta_state;
	}
	
	// x is m, v is m/s, a is m/s^2
	double leftx_k = 0.0;
	double leftv_k = 0.0;
	double lefta_k = 0.0;
	double rightx_k = 0.0;
	double rightv_k = 0.0;
	double righta_k = 0.0;
		
	private double leftr_k = 0.0;
	private double rightr_k = 0.0;
		
	private double leftx_k1 = 0.0;
	private double leftv_k1 = 0.0;
	private double lefta_k1 = 0.0;
	private double rightx_k1 = 0.0;
	private double rightv_k1 = 0.0;
	private double righta_k1 = 0.0;

	boolean useFrontLeftEncoder = true;
	boolean useBackLeftEncoder = true;
	boolean useFrontRightEncoder = true;
	boolean useBackRightEncoder = true;
	
	public void updateLR(double dT, double powerToLeft, double powerToRight, double frontLeftEncoderDistance, double backLeftEncoderDistance, double frontRightEncoderDistance, double backRightEncoderDistance) {
		leftx_k1 = leftx_k + (dT * leftv_k) + ((Math.pow(dT, 2) * lefta_k) / 2);
		leftv_k1 = leftv_k + (dT * lefta_k);
		lefta_k1 = powerToLeft;
			
		rightx_k1 = rightx_k + (dT * rightv_k) + ((Math.pow(dT, 2) * righta_k) / 2);
		rightv_k1 = rightv_k + (dT * righta_k);		
		righta_k1 = powerToRight;

		if (useBackLeftEncoder && useFrontLeftEncoder) {
			if (Math.abs(leftx_k1 - frontLeftEncoderDistance) / leftx_k1 < 0.02 && Math.abs(leftx_k1 - backLeftEncoderDistance) / leftx_k1 > 0.10) {
				useBackLeftEncoder = false;
				System.out.println("Not using back left encoder");
			} else if (Math.abs(leftx_k1 - backLeftEncoderDistance) / leftx_k1 < 0.02 && Math.abs(leftx_k1 - frontLeftEncoderDistance) / leftx_k1 > 0.10) {
				useFrontLeftEncoder = false;
				System.out.println("Not using front left encoder");
			} 
		}
			
		if (useBackRightEncoder && useFrontRightEncoder) {
			if (Math.abs(rightx_k1 - frontRightEncoderDistance) / rightx_k1 < 0.02 && Math.abs(rightx_k1 - backRightEncoderDistance) / rightx_k1 > 0.10) {
				useBackRightEncoder = false;
				System.out.println("Not using back right encoder");
			} else if (Math.abs(rightx_k1 - backRightEncoderDistance) / rightx_k1 < 0.02 && Math.abs(rightx_k1 - frontRightEncoderDistance) / rightx_k1 > 0.10) {
				useFrontRightEncoder = false;
				System.out.println("Not using front right encoder");	
			}	
		}

		if (useBackLeftEncoder && useFrontLeftEncoder) {
			leftr_k = ((frontLeftEncoderDistance + backLeftEncoderDistance) / 2) - leftx_k1;
		} else if (useFrontLeftEncoder && !useBackLeftEncoder) {
			leftr_k = frontLeftEncoderDistance - leftx_k1;
		} else if (useBackLeftEncoder && !useFrontLeftEncoder) {
			leftr_k = backLeftEncoderDistance - leftx_k1;
		}
			
		if (useBackRightEncoder && useFrontRightEncoder) {
			rightr_k = ((frontRightEncoderDistance + backRightEncoderDistance) / 2) - rightx_k1;
		} else if (useFrontRightEncoder && !useBackRightEncoder) {
			rightr_k = frontRightEncoderDistance - rightx_k1;
		} else if (useBackRightEncoder && !useFrontRightEncoder) {
			rightr_k = backRightEncoderDistance - rightx_k1;	
		}

		leftx_k1 = leftx_k1 + (Constants.ALPHA * leftr_k);
		leftv_k1 = leftv_k1 + ((Constants.BETA / dT) * leftr_k);

		rightx_k1 = rightx_k1 + (Constants.ALPHA * rightr_k);
		rightv_k1 = rightv_k1 + ((Constants.BETA / dT) * rightr_k);

		leftx_k = leftx_k1;
		leftv_k = leftv_k1;
		lefta_k = lefta_k1;
			
		rightx_k = rightx_k1;
		rightv_k = rightv_k1;
		righta_k = righta_k1;
	}

	public void setLRState(double leftx_state, double leftv_state, double lefta_state, double rightx_state, double rightv_state, double righta_state) {
		leftx_k = leftx_state;
		leftv_k = leftv_state;
		lefta_k = lefta_state;
		rightx_k = rightx_state;
		rightv_k = rightv_state;
		righta_k = righta_state;
	}
}