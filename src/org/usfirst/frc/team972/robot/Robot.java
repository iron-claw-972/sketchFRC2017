package org.usfirst.frc.team972.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;
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

	CANTalon frontLeftMotor = new CANTalon(1);
	CANTalon frontRightMotor = new CANTalon(3);
	CANTalon backLeftMotor = new CANTalon(2);
	CANTalon backRightMotor = new CANTalon(4);
	CANTalon winchMotor = new CANTalon(5);

	DoubleSolenoid piston = new DoubleSolenoid(4, 5);

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

		try {
			new Compressor(30).start();
		} catch (Exception e) {
			e.printStackTrace();
			System.out.println("COMPRESSOR FAILED!");
		}

		autoSelected = autoChooser.getSelected();

		System.out.println("Auto Selected: " + autoSelected);
		SmartDashboard.putString("Auto Selected", autoSelected);
	}

	public void autonomousPeriodic() {
		if (System.currentTimeMillis() - startTime < 2500) {
			rd.tankDrive(-0.7, -0.7);
		} else {
			rd.tankDrive(0, 0);
		}

		switch (autoSelected) {
			case TIME_MIDDLE_GEAR:

				break;
			case TIME_BASELINE:
			default:
				// Put default auto code here
				break;
		}
	}

	public void teleopInit() {
		CameraServer.getInstance().startAutomaticCapture();
		try {
			new Compressor(30).start();
		} catch (Exception e) {
			e.printStackTrace();
			System.out.println("COMPRESSOR FAILED!");
		}

		piston.set(DoubleSolenoid.Value.kReverse);
	}

	@Override
	public void teleopPeriodic() {

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
			leftSpeed *= 3.0/4.0;
			rightSpeed *= 3.0/4.0;
		} else if (mode == 2) {
			leftSpeed = Math.abs(leftSpeed) * leftSpeed;
			rightSpeed = Math.abs(rightSpeed) * rightSpeed;
		}
		
		rd.tankDrive(leftSpeed, rightSpeed);

		if (operatorJoystick.getRawButton(11) || gamepad.getRawButton(4)) {
			winchMotor.set(1.0);
		} else if (operatorJoystick.getRawButton(12) || gamepad.getRawButton(1)) {
			winchMotor.set(0.5);
		} else {
			winchMotor.set(0);
		}

	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
}
