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
	public final double ENCODER_CLICKS_PER_ROTATION = 2048;
	public final double ROBOT_DRIVE_WHEEL_CIRCUMFERENCE = 0.320;

	double prevTime = 0.0;
	
	double timeHolder = 0; //used as a temp value for time for stuff
	boolean newHeadingClick = false;
	
	double BASELINE_TIME = 0;
	double BASELINE_POWER = 0.5;
	
	double MIDDLE_GEAR_POWER = 0.5; //We drive at this power
	double MIDDLE_GEAR_DISTANCE = 100; //Until we get to this distance
	double MIDDLE_GEAR_PUSH_TIME = 0.5; //Then we push for this long
	double MIDDLE_GEAR_PUSH_POWER = 0.25; //With this much power
	double MIDDLE_GEAR_TIME_LIMIT = 5; //And if we go this long, we auto-stop (incase sensor breaks or somthing)
	
	CANTalon frontLeftMotor = new CANTalon(1);
	CANTalon frontRightMotor = new CANTalon(3);
	CANTalon backLeftMotor = new CANTalon(2);
	CANTalon backRightMotor = new CANTalon(4);
	CANTalon winchMotor = new CANTalon(5);
	
	Encoder leftDriveEncoderFront = new Encoder(0, 1, true, Encoder.EncodingType.k2X);
	Encoder rightDriveEncoderFront = new Encoder(2, 3, false, Encoder.EncodingType.k2X);
	Encoder leftDriveEncoderBack = new Encoder(4, 5, true, Encoder.EncodingType.k2X);
	Encoder rightDriveEncoderBack = new Encoder(6, 7, false, Encoder.EncodingType.k2X);

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

	final String BASELINE = "Baseline";
	final String MIDDLE_GEAR = "Middle Gear";
	final String LEFT_GEAR = "Left Gear";
	final String RIGHT_GEAR = "Right Gear";
	final String DO_NOTHING = "Do Nothing";
	String autoSelected;

	SendableChooser<String> autoChooser = new SendableChooser<>();
	
	TimeOfFlight tof;
	PIDControl pid;
	
	@Override
	public void robotInit() {
		IMU.init();
		autoChooser.addDefault("Baseline", BASELINE);
		autoChooser.addObject("Middle Gear", MIDDLE_GEAR);
		autoChooser.addObject("Left Gear", LEFT_GEAR);
		autoChooser.addObject("Right Gear", RIGHT_GEAR);
		autoChooser.addObject("Do Nothing", DO_NOTHING);
		SmartDashboard.putData("Auto choices", autoChooser);

		frontLeftMotor.enableBrakeMode(false);
		frontRightMotor.enableBrakeMode(false);
		backLeftMotor.enableBrakeMode(false);
		frontLeftMotor.enableBrakeMode(false);
	}

	public void autonomousInit() {
		
		try {
			if(tof != null) {
				tof.port.closePort();
			}
	    	tof = new TimeOfFlight();
		} catch(Exception e) {
			e.printStackTrace();
			System.out.println("TIME OF FLIGHT SENSOR COULD NOT WORK FOR SOME REASON.");
		}
		
		Time.init();

		leftDriveEncoderFront.reset();
		rightDriveEncoderFront.reset();
		leftDriveEncoderBack.reset();
		rightDriveEncoderBack.reset();
		
		IMU.recalibrate(0.0);
		
		try {
			new Compressor(30).start();
		} catch (Exception e) {
			e.printStackTrace();
			System.out.println("COMPRESSOR FAILED!");
		}

		autoSelected = autoChooser.getSelected();
		System.out.println("Auto Selected: " + autoSelected);
		SmartDashboard.putString("Auto Selected", autoSelected);
		piston.set(DoubleSolenoid.Value.kForward);
		
		switch (autoSelected) {
			case BASELINE:
		        pid = new PIDControl(0.012, 0.000, 0);
		        pid.setOutputLimits(-0.5, 0.5);
		        pid.setSetpoint(0);
				break;
			case MIDDLE_GEAR:
				break;
			case LEFT_GEAR:
				break;
			case RIGHT_GEAR:
				break;
			case DO_NOTHING:
				rd.tankDrive(0.0, 0.0);
			break;
		}
	}

	public void autonomousPeriodic() {
		double currTime = Time.get();
		double loopTime = currTime - prevTime;
		switch (autoSelected) {
			case BASELINE:
				if(loopTime < BASELINE_TIME) {
					double currentAngle = (IMU.getAngle());
					double pidOutputPower = pid.getOutput(currentAngle);
	            
					rd.tankDrive(BASELINE_POWER + (pidOutputPower/4), BASELINE_POWER + (-pidOutputPower/4));
				} else {
					rd.tankDrive(0, 0);
				}
				break;
			case MIDDLE_GEAR:
				
				if(currTime > MIDDLE_GEAR_TIME_LIMIT) {
					rd.tankDrive(0, 0);
					break; //We done!
				}
				
				if((tof.GetDataInMillimeters() <= MIDDLE_GEAR_DISTANCE) && (tof.GetDataInMillimeters() > 0)) {
					if(timeHolder == 0) timeHolder = currTime; //get the time when the motors go into push gear mode.
						
					if(currTime < (MIDDLE_GEAR_PUSH_TIME + timeHolder)) {
						double currentAngle = (IMU.getAngle());
						double pidOutputPower = pid.getOutput(currentAngle); 
						rd.tankDrive(MIDDLE_GEAR_PUSH_POWER + (pidOutputPower/4), MIDDLE_GEAR_PUSH_POWER + (-pidOutputPower/4));
					} else {
						rd.tankDrive(0, 0); //We done!
					}
					
					break;
	        	} else {
	        		
	        		double currentAngle = IMU.getAngle();
	            	double pidOutputPower = pid.getOutput(currentAngle);
	            
	            	rd.tankDrive(MIDDLE_GEAR_POWER + (pidOutputPower/4), MIDDLE_GEAR_POWER + (-pidOutputPower/4));
	        	}
				break;
			case LEFT_GEAR:
				break;
			case RIGHT_GEAR:
				break;
			case DO_NOTHING:
				rd.tankDrive(0.0, 0.0);
				break;
		}
	}

	public void teleopInit() {
		CameraServer.getInstance().startAutomaticCapture();
		
        pid = new PIDControl(0.005, 0.000, 0);
        pid.setOutputLimits(-0.5, 0.5);
        pid.setSetpoint(0);
        
        IMU.recalibrate(0.0);
        
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
			leftSpeed *= 3.0 / 4.0;
			rightSpeed *= 3.0 / 4.0;
		} else if (mode == 2) {
			leftSpeed = Math.abs(leftSpeed) * leftSpeed;
			rightSpeed = Math.abs(rightSpeed) * rightSpeed;
		}
		
		//System.out.println(leftDriveEncoderFront.get() + " " + rightDriveEncoderFront.get());
		if(gamepad.getRawAxis(3) > .3) {
			double pidOutputPower = pid.getOutput(IMU.getAngle());
			
        	if(newHeadingClick) {
        		IMU.recalibrate(0.0);
        		pid.reset();
        		newHeadingClick = false;
        	}
        	
            rd.tankDrive(rightSpeed + (pidOutputPower * .25), rightSpeed + (-pidOutputPower * .25));
		} else {
			rd.tankDrive(leftSpeed, rightSpeed);
			newHeadingClick = true;
		}

		if (operatorJoystick.getRawButton(11) || gamepad.getRawButton(4)) {
			winchMotor.set(1.0);
		} else if (operatorJoystick.getRawButton(12) || gamepad.getRawButton(1)) {
			winchMotor.set(0.5);
		} else {
			winchMotor.set(0);
		}
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
}