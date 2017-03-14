package org.usfirst.frc.team972.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotionProfiling {

	private int resetCounter = 0;

	SystemModel sm = new SystemModel();
	
	/*
	 * Updates robot model.
	 *
	 * @param dT						Loop time (change in time) in seconds
	 * @param gyro						Gyro angle in degrees
	 * @param x_accel					IMU x accel
	 * @param y_accel					IMU y accel
	 * @param frontLeftEncoderValue		Front left encoder distance
	 * @param backLeftEncoderValue		Back left encoder distance
	 * @param frontRightEncoderValue	Front right encoder distance
	 * @param backRightEncoderValue		Back right encoder distance
	 * @param LeftAccel					Left side acceleration (m/s^2)
	 * @param rightAccel				Right side acceleration (m/s^2)
	 */
	public void update(double dT, double gyro, double x_accel, double y_accel, double frontLeftEncoderValue, double backLeftEncoderValue,
			double frontRightEncoderValue, double backRightEncoderValue, double leftAccel, double rightAccel) {
		// @formatter:off
		sm.updateLR(dT, leftAccel, rightAccel, frontLeftEncoderValue * Constants.ROBOT_DRIVE_WHEEL_CIRCUMFERENCE / Constants.ENCODER_CLICKS_PER_ROTATION, 
				- backLeftEncoderValue * Constants.ROBOT_DRIVE_WHEEL_CIRCUMFERENCE / Constants.ENCODER_CLICKS_PER_ROTATION, frontRightEncoderValue * Constants.ROBOT_DRIVE_WHEEL_CIRCUMFERENCE / Constants.ENCODER_CLICKS_PER_ROTATION, 
				backRightEncoderValue * Constants.ROBOT_DRIVE_WHEEL_CIRCUMFERENCE / Constants.ENCODER_CLICKS_PER_ROTATION);
		sm.updateSys(gyro, x_accel, y_accel, dT);
		// @formatter:on
		Logger.log("Motion_Profiling_Data", "t=" + Time.get() + " x=" + getX() + " y=" + getY() + " theta=" + getTheta());
	}

	/**
	 * Initialize at start of match
	 * 
	 * @param x_init	Initial x position in meters
	 * @param y_init	Initial y position in meters
	 */
	public void init(double x_init, double y_init) {
		IMU.init();
		sm.setLRState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		sm.setSysState(x_init, y_init, 0.0, 0.0, 0.0);
		Logger.log("Motion_Profiling_Data", "Initializing motion profiling...");
		Logger.log("Motion_Profiling_Data", "t=" + Time.get() + " x=" + getX() + " y=" + getY() + " theta=" + getTheta());
	}

	/**
	 * Resets the robot position when at a fixed location (i.e. the gear lift)
	 * 
	 * @param x_pos	New x position in meters
	 * @param y_pos	New y position in meters
	 * @param theta	New angle in degrees
	 */
	public void reset(Robot r, double x_pos, double y_pos, double theta) {
		double distance = 0.5 * ((Math.PI * Constants.ROBOT_WIDTH) / 360) * theta;
		// this makes sure that our system model doesn't mess up if theta isn't zero
		
		double average_encoder_value = 0.0;
		if (!sm.useBackLeftEncoder) {
			if (!sm.useBackRightEncoder) {
				average_encoder_value = (r.leftDriveEncoderFront.get() + r.rightDriveEncoderFront.get()) / 2;
			} else if (!sm.useFrontRightEncoder) {
				average_encoder_value = (r.leftDriveEncoderFront.get() + r.rightDriveEncoderBack.get()) / 2;
			} else {
				average_encoder_value = (r.leftDriveEncoderFront.get() + r.rightDriveEncoderFront.get() + r.rightDriveEncoderBack.get()) / 3;
			}
		} else if (!sm.useFrontLeftEncoder) {
			if (!sm.useBackRightEncoder) {
				average_encoder_value = (r.leftDriveEncoderBack.get() + r.rightDriveEncoderFront.get()) / 2;
			} else if (!sm.useFrontRightEncoder) {
				average_encoder_value = (r.leftDriveEncoderBack.get() + r.rightDriveEncoderBack.get()) / 2;
			} else {
				average_encoder_value = (r.leftDriveEncoderBack.get() + r.rightDriveEncoderFront.get() + r.rightDriveEncoderBack.get()) / 3;
			}
		} else if (!sm.useBackRightEncoder) {
			average_encoder_value = (r.leftDriveEncoderFront.get() + r.rightDriveEncoderFront.get() + r.leftDriveEncoderBack.get()) / 3;
		} else if (!sm.useFrontRightEncoder) {
			average_encoder_value = (r.leftDriveEncoderFront.get() + r.rightDriveEncoderBack.get() + r.leftDriveEncoderBack.get()) / 3;
		} else {
			average_encoder_value = (r.leftDriveEncoderFront.get() + r.rightDriveEncoderBack.get() + r.leftDriveEncoderBack.get() + r.rightDriveEncoderFront.get()) / 4;
		}

		sm.setLRState(average_encoder_value + distance, 0.0, 0.0, average_encoder_value - distance, 0.0, 0.0);
		sm.setSysState(x_pos, y_pos, 0.0, 0.0, theta);
		IMU.recalibrate(0.0);
		resetCounter++;
		Logger.renameFile("Motion_Profiling_Data", "Motion_Profiling_Data_ResetNum" + resetCounter);
		Logger.log("Motion_Profiling_Data", "Resetting motion profiling...");
		Logger.log("Motion_Profiling_Data", "t=" + Time.get() + " x=" + getX() + " y=" + getY() + " theta=" + getTheta());
	}

	public void updateSmartDashboard() {
		SmartDashboard.putNumber("X Position", getX());
		SmartDashboard.putNumber("Y Position", getY());
		SmartDashboard.putNumber("Angle", getTheta());
		SmartDashboard.putNumber("Velocity", Math.pow((Math.pow(getV_X(), 2) + Math.pow(getV_Y(), 2)), 0.5));
		SmartDashboard.putNumber("Accel", Math.pow((Math.pow(getA_X(), 2) + Math.pow(getA_Y(), 2)), 0.5));
		System.out.println("X: " + getX() + "Y: " + getY() + "Angle: " + getTheta() + "V: " + Math.pow((Math.pow(getV_X(), 2) + Math.pow(getV_Y(), 2)), 0.5) + "A: " + Math.pow((Math.pow(getA_X(), 2) + Math.pow(getA_Y(), 2)), 0.5));
	}

	public double getX() {
		return sm.x_k;
	}

	public double getY() {
		return sm.y_k;
	}

	public double getV_X() {
		return sm.v_xk;
	}
	
	public double getV_Y() {
		return sm.v_yk;
	}
	
	public double getA_X() {
		return sm.a_xk;	
	}
	
	public double getA_Y() {
		return sm.a_yk;
	}

	public double getTheta() {
		return sm.theta_k;
	}
}