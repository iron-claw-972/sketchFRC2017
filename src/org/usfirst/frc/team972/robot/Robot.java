package org.usfirst.frc.team972.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	boolean useMiddleGearAuto = false;
	boolean visionData = false;
	double gear_x;
	double gear_y;
	double gear_theta;
	double prevTime = 0.0;
	double prev_v_error = 0.0;
	double prev_theta_error = 0.0;

	CANTalon frontLeftMotor = new CANTalon(1);
	CANTalon frontRightMotor = new CANTalon(3);
	CANTalon backLeftMotor = new CANTalon(2);
	CANTalon backRightMotor = new CANTalon(4);
	CANTalon winchMotor = new CANTalon(5);

	Encoder leftDriveEncoderFront = new Encoder(0, 1, true, Encoder.EncodingType.k2X);
	Encoder rightDriveEncoderFront = new Encoder(2, 3, false, Encoder.EncodingType.k2X);
	Encoder leftDriveEncoderBack = new Encoder(4, 5, true, Encoder.EncodingType.k2X);
	Encoder rightDriveEncoderBack = new Encoder(6, 7, false, Encoder.EncodingType.k2X);
	
	MotionProfiling mot = new MotionProfiling();
	
//	CANTalon shooterMotorA = new CANTalon(6);
//	CANTalon shooterMotorB = new CANTalon(7);
//	CANTalon azimuthMotor = new CANTalon(10);

	Joystick gamepad = new Joystick(1);
	Joystick operatorJoystick = new Joystick(0);
	RobotDrive rd = new RobotDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

	boolean winchPressedLastTime = false;
	boolean runWinch = false;

	long startTime = 0;

	@Override
	public void robotInit() {
		IMU.init();
	}

	public void autonomousInit() {
		startTime = System.currentTimeMillis();
		Time.init();
		if (useMiddleGearAuto) {
			mot.init(Constants.MIDDLE_GEAR_AUTO_STARTX, Constants.MIDDLE_GEAR_AUTO_STARTY);
			Vision.startGearVision();
		}
	}

	public void autonomousPeriodic() {
		double currTime = Time.get();
		double loopTime = currTime - prevTime;
		if (useMiddleGearAuto) {
			boolean done = false;
			if (visionData) {
				if (Vision.newData()) {
					double distance = Vision.getDistance();
					double angle = Vision.getAngle() - 90; //change from 0 to 180 to -90 to 90
					double data_time = Vision.getTime();
					double[] framePosition = Logger.readLog("Motion_Profiling_Data", data_time);
					if (framePosition.length == 3) {
						gear_x = Math.sin(angle) * distance + Math.sin(framePosition[2]) * (Constants.ROBOT_LENGTH / 2) + framePosition[0];
						gear_y = Math.cos(angle) * distance + Math.cos(framePosition[2]) * (Constants.ROBOT_LENGTH / 2) + framePosition[1];
						gear_theta = framePosition[2] + angle;
						if (framePosition[2] > 90 && angle > 90) {
							gear_theta = gear_theta - 360; 
						} else if (framePosition[2] < -90 && angle < -90) {
							gear_theta = gear_theta + 360;
						}
					} else {
						Logger.logError("Failed to determine the position of the robot from the logs.");
					}
				}
				if (gear_x != 0.0 && gear_y != 0.0) {
					done = autonDrive(gear_x, gear_y - Constants.LENGTH_GEAR_PEG - (Constants.ROBOT_LENGTH / 2), gear_theta, loopTime);
				}
			} else {
				visionData = Vision.newData();
				done = autonDrive(Constants.MIDDLE_GEAR_AUTO_X, Constants.MIDDLE_GEAR_AUTO_Y - (Constants.ROBOT_LENGTH / 2), Constants.MIDDLE_GEAR_AUTO_THETA, loopTime);
			}
			if (done) {
				mot.reset(Constants.MIDDLE_GEAR_AUTO_X, Constants.MIDDLE_GEAR_AUTO_Y - (Constants.ROBOT_LENGTH / 2), Constants.MIDDLE_GEAR_AUTO_THETA);
				gear_x = 0.0;
				gear_y = 0.0;
				gear_theta = 0.0;
			}
			updateModel(loopTime);
			prevTime = currTime;
		} else {
			if (System.currentTimeMillis() - startTime < 2500) {
				rd.tankDrive(-0.7, -0.7);
			} else {
				rd.tankDrive(0, 0);
			}
		}
	}

	public void teleopInit() {
		CameraServer.getInstance().startAutomaticCapture();
//		shooterMotorB.changeControlMode(TalonControlMode.Follower);
//		shooterMotorB.set(6);
	}

	@Override
	public void teleopPeriodic() {
		rd.tankDrive(gamepad.getRawAxis(5), gamepad.getRawAxis(1));

		boolean winchPressed = operatorJoystick.getRawButton(11);

		if (winchPressed) {
			winchMotor.set(1.0);
		} else if (operatorJoystick.getRawButton(12)) {
			winchMotor.set(0.5);
		} else {
			winchMotor.set(0);
		}

//		if (operatorJoystick.getRawButton(1)) {
//			shooterMotorA.set((1.0 - operatorJoystick.getThrottle()) / 2.0);
//		}
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
	
	public boolean autonDrive(double x_desired, double y_desired, double theta_desired, double dT) {
		double curr_x = mot.getX();
		double curr_y = mot.getY();
		double curr_v = Math.pow((Math.pow(mot.getV_X(), 2) + Math.pow(mot.getV_Y(), 2)), 0.5);
		double curr_theta = mot.getTheta();
		
		double v_error = 0.0;
		double theta_error = 0.0;
		
		boolean done = false;
		
		// distance from robot to desired point using Pythagorean theorem
		double distance = Math.pow((Math.pow((x_desired - curr_x), 2) + Math.pow((y_desired - curr_y), 2)), 0.5);
		if (distance > 0.12) { //maybe more
			//get trajectory angle from -pi to pi but with 0 on the x axis
			double trajectory_angle = 0.0;
			if (x_desired >= curr_x && y_desired >= curr_y) {
				 trajectory_angle = Math.abs(Math.atan((y_desired - curr_y) / (x_desired - curr_x)));
			} else if (x_desired <= curr_x && y_desired >= curr_y) {
				 trajectory_angle = Math.PI - Math.abs(Math.atan((y_desired - curr_y) / (x_desired - curr_x)));
			} else if (x_desired >= curr_x && y_desired <= curr_y) {
				trajectory_angle = - Math.abs(Math.atan((y_desired - curr_y) / (x_desired - curr_x)));
			} else if (x_desired <= curr_x && y_desired <= curr_y) {
				trajectory_angle = - Math.PI + Math.abs(Math.atan((y_desired - curr_y) / (x_desired - curr_x)));
			}
			
			//convert angle to degrees in the -180 to 180 scheme used everywhere
			if (trajectory_angle > Math.PI / 2) {
				trajectory_angle = (180 * trajectory_angle / Math.PI) - 90;
			} else if (trajectory_angle <  - Math.PI / 2) {
				trajectory_angle = - 270 - (180 * trajectory_angle / Math.PI);
			} else {
				trajectory_angle = 90 - (180 * trajectory_angle / Math.PI);
			}
			SmartDashboard.putNumber("traj_angle", trajectory_angle);
			SmartDashboard.putNumber("dist", distance);
			
			theta_error = - trajectory_angle + curr_theta;
			if (trajectory_angle > 90 && curr_theta < -90) {
				theta_error = theta_error + 360; 
			} else if (trajectory_angle < -90 && curr_theta > 90) {
				theta_error = theta_error - 360;
			}
			
			boolean isReverseDrive = false;
			if (theta_error > 90.0) {
				isReverseDrive = true;
				theta_error = 180 - theta_error;
			} else if (theta_error < -90.0) {
				isReverseDrive = true;
				theta_error = -theta_error - 180;
			}
			
			v_error = Constants.ROBOT_MAX_VELOCITY - curr_v;
			if (distance < Constants.AUTON_STOPPING_DISTANCE_2) {
				v_error = ((Constants.AUTON_STOPPING_DISTANCE_2 - distance) *
						((Constants.ROBOT_MAX_VELOCITY * Constants.AUTON_VELOCITY_STOPPING_PROPORTION) / Constants.AUTON_STOPPING_DISTANCE_2)) - curr_v;
			} else if (distance < Constants.AUTON_STOPPING_DISTANCE_1) {
				v_error = ((Constants.AUTON_STOPPING_DISTANCE_1 - distance) * 
						(((1 - Constants.AUTON_VELOCITY_STOPPING_PROPORTION) * Constants.ROBOT_MAX_VELOCITY) / 
								(Constants.AUTON_STOPPING_DISTANCE_1 - Constants.AUTON_STOPPING_DISTANCE_2))) - curr_v;
			}
			
			double dVdT = 0.0;
			if (prev_v_error != 0.0) {
				dVdT = (v_error - prev_v_error) / dT;
			}
			double dThetadT = 0.0;
			if (prev_theta_error != 0.0) {
				dThetadT = (theta_error - prev_theta_error) / dT;
			}
			
			double power_for_velocity = (Constants.AUTON_DRIVE_RATIO - Constants.AUTON_DRIVE_F) * ((Constants.AUTON_DRIVE_VP * v_error) - (Constants.AUTON_DRIVE_VD * dVdT));
			double power_for_turning = (1 - Constants.AUTON_DRIVE_RATIO) * ((Constants.AUTON_DRIVE_AP * theta_error) - 
					(Constants.AUTON_DRIVE_AD * dThetadT));
			
			double leftDriveInput = power_for_velocity + power_for_turning + Constants.AUTON_DRIVE_F;
			double rightDriveInput = power_for_velocity - power_for_turning + Constants.AUTON_DRIVE_F;
		
			if (isReverseDrive) {
				rd.tankDrive(-leftDriveInput, -rightDriveInput);
			} else {
				rd.tankDrive(leftDriveInput, rightDriveInput);
			}
		} else {
			if (curr_v > 0.15) {
				rd.tankDrive(curr_v / 15, curr_v / 15);
			} else {
				theta_error = - theta_desired + curr_theta;
				if (theta_desired > 90 && curr_theta < -90) {
					theta_error = theta_error + 360; 
				} else if (theta_desired < -90 && curr_theta > 90) {
					theta_error = theta_error - 360;
				}
				double dThetadT = 0.0;
				if (prev_theta_error != 0.0) {
						dThetadT = ( - theta_error + prev_theta_error) / dT;
				}
				double turn_power = (Constants.AUTON_DRIVE_TURNP * theta_error) - (Constants.AUTON_DRIVE_TURND * dThetadT);
				
				double leftDriveInput = turn_power;
				double rightDriveInput = - turn_power;
				if (turn_power > 0) {
					leftDriveInput = leftDriveInput + Constants.AUTON_DRIVE_TURNF;
					rightDriveInput = rightDriveInput - Constants.AUTON_DRIVE_TURNF;
				} else {
					leftDriveInput = leftDriveInput - Constants.AUTON_DRIVE_TURNF;
					rightDriveInput = rightDriveInput + Constants.AUTON_DRIVE_TURNF;
				}
				
				if (Math.abs(theta_error) < 7 && Math.abs(dThetadT) < 2) {
					done = true;
					prev_v_error = 0.0;
					prev_theta_error = 0.0;
				} else {			
					rd.tankDrive(leftDriveInput, rightDriveInput);
				}
			}
		}
		
		SmartDashboard.putNumber("V Error", v_error);
		SmartDashboard.putNumber("Theta Error", theta_error);
		
		prev_v_error = v_error;
		prev_theta_error = theta_error;
		
		if (done) {
			rd.tankDrive(0.0, 0.0);
		}
		
		return done;
	}
	
	public void updateModel(double dT) {
		mot.update(dT, IMU.getAngle(), IMU.getAccelX(), IMU.getAccelY(), leftDriveEncoderFront.get(), leftDriveEncoderBack.get(), rightDriveEncoderFront.get(), 
				rightDriveEncoderBack.get(), getAccel("left"), getAccel("right"));
	}

	/**
	 * Gets acceleration of a robot side.
	 * 
	 * @param robotSide
	 *            the side to get acceleration of ("left" or "right")
	 * @return acceleration of the side using system model
	 */
	public double getAccel(String robotSide) {
		if (robotSide == "left") {
			return 0.4448 * ((frontLeftMotor.getOutputCurrent() + backLeftMotor.getOutputCurrent()) / 2) * (1 - 0.0356 * SystemModel.leftv_k) / Constants.ROBOT_MASS;
		} else if (robotSide == "right") {
			return 0.4448 * ((frontRightMotor.getOutputCurrent() + backRightMotor.getOutputCurrent()) / 2) * (1 - 0.0356 * SystemModel.rightv_k) / Constants.ROBOT_MASS;
		} else {
			return -9001.0;
		}
	}
}
