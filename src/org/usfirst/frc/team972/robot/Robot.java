package org.usfirst.frc.team972.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the IterativeRobot documentation. If you change the name of this class
 * or the package after creating this project, you must also update the manifest file in the
 * resource directory.
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

	public final double METERS_FOR_ROBOT_TO_TRAVEL_POR_EL_GEAR = .71;			

	public final double ENCODER_CLICKS_PER_ROTATION = 2048;
	public final double ROBOT_DRIVE_WHEEL_CIRCUMFERENCE = 0.320;

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
	SystemModel sys = new SystemModel();

	DoubleSolenoid piston = new DoubleSolenoid(30, 0, 1);

	Joystick gamepad = new Joystick(1);
	Joystick operatorJoystick = new Joystick(0);
	RobotDrive rd = new RobotDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

	boolean winchPressedLastTime = false;
	boolean runWinch = false;

	int mode = 0;

	double leftSpeed = 0.0;
	double rightSpeed = 0.0;

	long startTime = 0;

	boolean runTimeBaseline = false, runTimeMidGear = false, runEncoderMidGear = false, runMotionProfiling = false;

	final String TIME_BASELINE = "Time Baseline";
	final String TIME_MIDDLE_GEAR = "Time Middle Gear";
	final String ENCODER_MIDDLE_GEAR = "Encoder Middle Gear";
	final String MOTION_PROFILING = "Motion Profiling";
	final String DO_NOTHING = "Do Nothing";
	String autoSelected;

	SendableChooser<String> autoChooser = new SendableChooser<>();

	@Override
	public void robotInit() {
		IMU.init();
		autoChooser.addDefault("Time Baseline", TIME_BASELINE);
		autoChooser.addObject("Time Middle Gear", TIME_MIDDLE_GEAR);
		autoChooser.addObject("Encoder Middle Gear", ENCODER_MIDDLE_GEAR);
		autoChooser.addObject("Motion Profiling", MOTION_PROFILING);
		autoChooser.addObject("Do Nothing", DO_NOTHING);
		SmartDashboard.putData("Auto choices", autoChooser);

		frontLeftMotor.enableBrakeMode(false);
		frontRightMotor.enableBrakeMode(false);
		backLeftMotor.enableBrakeMode(false);
		frontLeftMotor.enableBrakeMode(false);
	}

	public void autonomousInit() {
		startTime = System.currentTimeMillis();
		Time.init();
		if (useMiddleGearAuto) {
			mot.init(Constants.MIDDLE_GEAR_AUTO_STARTX, Constants.MIDDLE_GEAR_AUTO_STARTY);
			Vision.startGearVision();
		}

		leftDriveEncoderFront.reset();
		rightDriveEncoderFront.reset();
		
    try {
			new Compressor(30).start();
		} catch (Exception e) {
			e.printStackTrace();
			System.out.println("COMPRESSOR FAILED!");
		}

//		autoSelected = autoChooser.getSelected();

		autoSelected = "Encoder Middle Gear";
		System.out.println("Auto Selected: " + autoSelected);
		SmartDashboard.putString("Auto Selected", autoSelected);
		piston.set(DoubleSolenoid.Value.kForward);
		mot.updateSmartDashboard();
	}

	public void autonomousPeriodic() {
		piston.set(DoubleSolenoid.Value.kForward);//forward = pistons down
		double currTime = Time.get();
		double loopTime = currTime - prevTime;
		updateModel(loopTime);
		prevTime = currTime;
		autonDrive(0.0, 1.7, 0.0, loopTime);
		/*if (useMiddleGearAuto) {
			boolean done = false;
			/*if (visionData) {
				if (Vision.newData()) {
					double distance = Vision.getDistance();
					double angle = Vision.getAngle() - 90; // change from 0 to 180 to -90 to 90
					double data_time = Vision.getTime();
					double[] framePosition = Logger.readLog("Motion_Profiling_Data", data_time);
					if (framePosition.length == 3) {
						gear_x = Math.sin(angle) * distance + Math.sin(framePosition[2]) * (Constants.ROBOT_LENGTH / 2)
								+ framePosition[0];
						gear_y = Math.cos(angle) * distance + Math.cos(framePosition[2]) * (Constants.ROBOT_LENGTH / 2)
								+ framePosition[1];
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
					done = autonDrive(gear_x, gear_y - Constants.LENGTH_GEAR_PEG,
							gear_theta, loopTime);
				}
			} else {
				visionData = Vision.newData();*/
				/*done = autonDrive(Constants.MIDDLE_GEAR_AUTO_X,
						Constants.MIDDLE_GEAR_AUTO_Y, Constants.MIDDLE_GEAR_AUTO_THETA,
						loopTime);*/
			//}
		/*	if (done) {
				mot.reset(this, Constants.MIDDLE_GEAR_AUTO_X,
						Constants.MIDDLE_GEAR_AUTO_Y, Constants.MIDDLE_GEAR_AUTO_THETA);
				gear_x = 0.0;
				gear_y = 0.0;
				gear_theta = 0.0;
			}
			updateModel(loopTime);
			prevTime = currTime;
		}*//*else {
			if (autoSelected == "Encoder Middle Gear") {
				boolean pastPoint = ((rightDriveEncoderFront
						.get() < ticksFromMeters(METERS_FOR_ROBOT_TO_TRAVEL_POR_EL_GEAR))
						|| (leftDriveEncoderFront.get() < ticksFromMeters(METERS_FOR_ROBOT_TO_TRAVEL_POR_EL_GEAR)));
				if ((System.currentTimeMillis() - startTime < 3000) && pastPoint) {
					rd.tankDrive(-0.722, -0.7); // (right, left)
				} else {
					System.out.println("AUTO FINISHED Traveled:" + leftDriveEncoderFront.get() + " : " + rightDriveEncoderFront.get());
					//System.out.println(leftDriveEncoderBack.get() + " : " + rightDriveEncoderBack.get());
					System.out.println("Time: " + (System.currentTimeMillis() - startTime));
					rd.tankDrive(0, 0);
				}
			} else if (autoSelected == "Time Middle Gear") {
				if (System.currentTimeMillis() - startTime < 1920) {
					rd.tankDrive(-0.7, -0.7);
				} else {
					rd.tankDrive(0, 0);
				}
			} else if (autoSelected == "Time Baseline") {
				if (System.currentTimeMillis() - startTime < 2500) {
					rd.tankDrive(-0.7, -0.7);
				} else {
					rd.tankDrive(0, 0);
				}
			}
		}

		switch (autoSelected) {
			case TIME_MIDDLE_GEAR:

				break;
			case TIME_BASELINE:
			default:
				// Put default auto code here
				break;
		} */
		mot.updateSmartDashboard();
	}

	public void teleopInit() {
		piston.set(DoubleSolenoid.Value.kForward);//forward = pistons down
		mot.init(0.0, 0.0); //TODO remove
		Time.init();
		CameraServer.getInstance().startAutomaticCapture();
		try {
			new Compressor(30).start();
		} catch (Exception e) {
			e.printStackTrace();
			System.out.println("COMPRESSOR FAILED!");
		}

		piston.set(DoubleSolenoid.Value.kForward);
	}
	
	public void disabledPeriodic() {
		piston.set(DoubleSolenoid.Value.kReverse);
		mot.updateSmartDashboard();
	}

	@Override
	public void teleopPeriodic() {

		if(operatorJoystick.getRawButton(5)) {//TODO: Change button number to button preferred by operator
			piston.set(DoubleSolenoid.Value.kReverse);//in case gear gets stuckon top of the gear mech, it can unjam
		}
		else {
			piston.set(DoubleSolenoid.Value.kForward);
		}
		
		if (gamepad.getRawButton(5)) { // left top
			mode = 0; // regular
		} else if (gamepad.getRawButton(6)) { // right top
			mode = 1; // 3/4 speed
		} else if (gamepad.getRawButton(2)) { // B button
			mode = 2; // squared
		}

		leftSpeed = gamepad.getRawAxis(5);
		rightSpeed = gamepad.getRawAxis(1);

		if (mode == 0) {
		} else if (mode == 1) {
			leftSpeed *= 3.0 / 4.0;
			rightSpeed *= 3.0 / 4.0;
		} else if (mode == 2) {
			leftSpeed = Math.abs(leftSpeed) * leftSpeed;
			rightSpeed = Math.abs(rightSpeed) * rightSpeed;
		}
		
		//System.out.println(leftDriveEncoderFront.get() + " " + rightDriveEncoderFront.get());
		rd.tankDrive(leftSpeed, rightSpeed);

		if (operatorJoystick.getRawButton(11) || gamepad.getRawButton(4)) {
			winchMotor.set(1.0);
		} else if (operatorJoystick.getRawButton(12) || gamepad.getRawButton(1)) {
			winchMotor.set(0.5);
		} else {
			winchMotor.set(0);
		}
		double currTime = Time.get();
		double loopTime = currTime - prevTime;
		updateModel(loopTime);
		prevTime = currTime;
		mot.updateSmartDashboard();
	}

	public double ticksFromMeters(double meters) {
		return (ENCODER_CLICKS_PER_ROTATION / ROBOT_DRIVE_WHEEL_CIRCUMFERENCE) * meters;
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
		if (distance > 0.12) { // maybe more
			// get trajectory angle from -pi to pi but with 0 on the x axis
			double trajectory_angle = 0.0;
			if (x_desired >= curr_x && y_desired >= curr_y) {
				trajectory_angle = Math.abs(Math.atan((y_desired - curr_y) / (x_desired - curr_x)));
			} else if (x_desired <= curr_x && y_desired >= curr_y) {
				trajectory_angle = Math.PI - Math.abs(Math.atan((y_desired - curr_y) / (x_desired - curr_x)));
			} else if (x_desired >= curr_x && y_desired <= curr_y) {
				trajectory_angle = -Math.abs(Math.atan((y_desired - curr_y) / (x_desired - curr_x)));
			} else if (x_desired <= curr_x && y_desired <= curr_y) {
				trajectory_angle = -Math.PI + Math.abs(Math.atan((y_desired - curr_y) / (x_desired - curr_x)));
			}

			// convert angle to degrees in the -180 to 180 scheme used everywhere
			if (trajectory_angle > Math.PI / 2) {
				trajectory_angle = (180 * trajectory_angle / Math.PI) - 90;
			} else if (trajectory_angle < -Math.PI / 2) {
				trajectory_angle = -270 - (180 * trajectory_angle / Math.PI);
			} else {
				trajectory_angle = 90 - (180 * trajectory_angle / Math.PI);
			}
			SmartDashboard.putNumber("traj_angle", trajectory_angle);
			SmartDashboard.putNumber("dist", distance);

			theta_error = -trajectory_angle + curr_theta;
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
				v_error = ((Constants.AUTON_STOPPING_DISTANCE_2 - distance)
						* ((Constants.ROBOT_MAX_VELOCITY * Constants.AUTON_VELOCITY_STOPPING_PROPORTION)
								/ Constants.AUTON_STOPPING_DISTANCE_2))
						- curr_v;
			} else if (distance < Constants.AUTON_STOPPING_DISTANCE_1) {
				v_error = ((Constants.AUTON_STOPPING_DISTANCE_1 - distance)
						* (((1 - Constants.AUTON_VELOCITY_STOPPING_PROPORTION) * Constants.ROBOT_MAX_VELOCITY)
								/ (Constants.AUTON_STOPPING_DISTANCE_1 - Constants.AUTON_STOPPING_DISTANCE_2)))
						- curr_v;
			}

			double dVdT = 0.0;
			if (prev_v_error != 0.0) {
				dVdT = (v_error - prev_v_error) / dT;
			}
			double dThetadT = 0.0;
			if (prev_theta_error != 0.0) {
				dThetadT = (theta_error - prev_theta_error) / dT;
			}

			double power_for_velocity = (Constants.AUTON_DRIVE_RATIO - Constants.AUTON_DRIVE_F)
					* ((Constants.AUTON_DRIVE_VP * v_error) - (Constants.AUTON_DRIVE_VD * dVdT));
			double power_for_turning = (1 - Constants.AUTON_DRIVE_RATIO)
					* ((Constants.AUTON_DRIVE_AP * theta_error) - (Constants.AUTON_DRIVE_AD * dThetadT));

			double leftDriveInput = power_for_velocity + power_for_turning + Constants.AUTON_DRIVE_F;
			double rightDriveInput = power_for_velocity - power_for_turning + Constants.AUTON_DRIVE_F;

			if (isReverseDrive) {
				rd.tankDrive(-leftDriveInput, -rightDriveInput);
			} else {
				rd.tankDrive(leftDriveInput, rightDriveInput);
			}
			
			SmartDashboard.putNumber("LeftPower", leftDriveInput);
			SmartDashboard.putNumber("RightPower", rightDriveInput);
		} else {
			if (curr_v > 0.15) {
				rd.tankDrive(curr_v / 15, curr_v / 15);
			} else {
				theta_error = -theta_desired + curr_theta;
				if (theta_desired > 90 && curr_theta < -90) {
					theta_error = theta_error + 360;
				} else if (theta_desired < -90 && curr_theta > 90) {
					theta_error = theta_error - 360;
				}
				double dThetadT = 0.0;
				if (prev_theta_error != 0.0) {
					dThetadT = (-theta_error + prev_theta_error) / dT;
				}
				double turn_power = (Constants.AUTON_DRIVE_TURNP * theta_error)
						- (Constants.AUTON_DRIVE_TURND * dThetadT);

				double leftDriveInput = turn_power;
				double rightDriveInput = -turn_power;
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

				SmartDashboard.putNumber("LeftPower", leftDriveInput);
				SmartDashboard.putNumber("RightPower", rightDriveInput);
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
		mot.update(dT, IMU.getAngle(), IMU.getAccelX(), IMU.getAccelY(), leftDriveEncoderFront.get(),
				leftDriveEncoderBack.get(), rightDriveEncoderFront.get(), rightDriveEncoderBack.get(), getAccel("left"),
				getAccel("right"));
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
			return 0.4448 * ((frontLeftMotor.getOutputCurrent() + backLeftMotor.getOutputCurrent()) / 2)
					* (1 - 0.0356 * sys.leftv_k) / Constants.ROBOT_MASS;
		} else if (robotSide == "right") {
			return 0.4448 * ((frontRightMotor.getOutputCurrent() + backRightMotor.getOutputCurrent()) / 2)
					* (1 - 0.0356 * sys.rightv_k) / Constants.ROBOT_MASS;
		} else {
			return -9001.0;
		}
	}
	
	public void disabledInit(){
		piston.set(DoubleSolenoid.Value.kReverse);//reverse = pistons up
	}
	
}
