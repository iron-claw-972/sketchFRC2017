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

	// CANTalon shooterMotorA = new CANTalon(6);
	// CANTalon shooterMotorB = new CANTalon(7);
	// CANTalon azimuthMotor = new CANTalon(10);

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
	final String timeBaseline = "Time Baseline";
	final String timeMidGear = "Time Middle Gear";
	final String encoderMidGear = "Encoder Middle Gear";
	final String motionProfilingRun = "Motion Profiling";
	final String doNothing = "Do Nothing";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();

	@Override
	public void robotInit() {
		frontLeftMotor.enableBrakeMode(true);
		frontRightMotor.enableBrakeMode(true);
		backLeftMotor.enableBrakeMode(true);
		frontLeftMotor.enableBrakeMode(true);
//		chooser.addDefault("Do Nothing", doNothing);
		chooser.addDefault("Time Baseline", timeBaseline);
		chooser.addObject("Time Middle Gear", timeMidGear);
		chooser.addObject("Encoder Middle Gear", encoderMidGear);
		chooser.addObject("motionProfilingRun", motionProfilingRun);
		SmartDashboard.putData("Auto choices", chooser);
	}

	public void autonomousInit() {
		startTime = System.currentTimeMillis();

		try {
			new Compressor(30).start();
		} catch (Exception e) {
			e.printStackTrace();
			System.out.println("COMPRESSOR FAILED!");
		}

		autoSelected = chooser.getSelected();
//		autoSelected = SmartDashboard.getString("Auto Selector", timeBaseline);
		System.out.println("Auto selected: " + autoSelected);
	}

	public void autonomousPeriodic() {
		if (System.currentTimeMillis() - startTime < 2500) {
			rd.tankDrive(-0.7, -0.7);
		} else {
			rd.tankDrive(0, 0);
		}

		switch (autoSelected) {
			case timeMidGear:

				break;
			case timeBaseline:
			default:
				// Put default auto code here
				break;
		}
	}

	public void teleopInit() {
		CameraServer.getInstance().startAutomaticCapture();
		// shooterMotorB.changeControlMode(TalonControlMode.Follower);
		// shooterMotorB.set(6);
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
			mode = 1; // half
		} else if (gamepad.getRawButton(2)) { // B button
			mode = 2;
		}

		if (mode == 0) {
			leftSpeed = gamepad.getRawAxis(5);
			rightSpeed = gamepad.getRawAxis(1);
		} else if (mode == 1) {
			leftSpeed = gamepad.getRawAxis(5) / 2;
			rightSpeed = gamepad.getRawAxis(1) / 2;
		} else if (mode == 2) {
			leftSpeed = gamepad.getRawAxis(5) * Math.abs(gamepad.getRawAxis(5));
			rightSpeed = gamepad.getRawAxis(1) * Math.abs(gamepad.getRawAxis(1));
		}
		rd.tankDrive(leftSpeed, rightSpeed);

		if (operatorJoystick.getRawButton(11) || gamepad.getRawButton(4)) {
			winchMotor.set(1.0);
		} else if (operatorJoystick.getRawButton(12) || gamepad.getRawButton(1)) {
			winchMotor.set(0.5);
		} else {
			winchMotor.set(0);
		}

		// if (operatorJoystick.getRawButton(1)) {
		// shooterMotorA.set((1.0 - operatorJoystick.getThrottle()) / 2.0);
		// }
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
}
