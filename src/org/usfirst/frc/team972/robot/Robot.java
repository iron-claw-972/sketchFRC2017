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
	double timeHolder = 0; //used as a temp value for time for stuff
	boolean newHeadingClick = false;
	
	CANTalon frontLeftMotor = new CANTalon(1);
	CANTalon frontRightMotor = new CANTalon(3);
	CANTalon backLeftMotor = new CANTalon(2);
	CANTalon backRightMotor = new CANTalon(4);
	CANTalon winchMotorA = new CANTalon(5);
	CANTalon winchMotorB = new CANTalon(6);
	
	RockefellerEncoder leftDriveEncoderFront = new RockefellerEncoder(0, 1, true);
	RockefellerEncoder rightDriveEncoderFront = new RockefellerEncoder(2, 3, false);
	RockefellerEncoder leftDriveEncoderBack = new RockefellerEncoder(4, 5, true);
	RockefellerEncoder rightDriveEncoderBack = new RockefellerEncoder(6, 7, false);

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
	PIDControl driveDistancePid;
	
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
		
		leftDriveEncoderFront.reset();
		leftDriveEncoderBack.reset();
		rightDriveEncoderFront.reset();
		rightDriveEncoderBack.reset();
		
		winchMotorB.changeControlMode(TalonControlMode.Follower);
		winchMotorB.set(winchMotorA.getDeviceID());
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
		        pid = new PIDControl(Constants.TURNP, Constants.TURNI, Constants.TURND);
		        pid.setOutputLimits(Constants.PID_OUTPUT_LIMIT);
		        pid.setSetpoint(0);
		        
		        driveDistancePid = new PIDControl(Constants.DISTP, Constants.DISTI, Constants.DISTD, Constants.DISTF);
		        driveDistancePid.setOutputLimits(Constants.PID_OUTPUT_LIMIT);
		        driveDistancePid.setSetpoint(0);
		        driveDistancePid.setOutputRampRate(Constants.PID_RAMP_RATE);
				break;
			case MIDDLE_GEAR:
		        pid = new PIDControl(Constants.TURNP, Constants.TURNI, Constants.TURND);
		        pid.setOutputLimits(Constants.PID_OUTPUT_LIMIT);
		        pid.setSetpoint(0);
				break;
			case LEFT_GEAR:
				pid = new PIDControl(Constants.TURNP, Constants.TURNI, Constants.TURND);
		        pid.setOutputLimits(Constants.PID_OUTPUT_LIMIT);
		        pid.setSetpoint(0);
				break;
			case RIGHT_GEAR:
				pid = new PIDControl(Constants.TURNP, Constants.TURNI, Constants.TURND);
		        pid.setOutputLimits(Constants.PID_OUTPUT_LIMIT);
		        pid.setSetpoint(0);
				break;
			case DO_NOTHING:
				rd.tankDrive(0.0, 0.0);
			break;
		}
	}

	int InRange = 0;
	
	public void autonomousPeriodic() {
		double currTime = Time.get();
		switch (autoSelected) {
			case BASELINE:
				if(InRange < (50 * 1)) { //50 loops per second
					double distanceAway = Constants.BASELINE_DISTANCE - ((rightDriveEncoderFront.getDistance() + leftDriveEncoderFront.getDistance())/2);
					double pidDriveBasePower = driveDistancePid.getOutput(distanceAway);
					
					if(Math.abs(distanceAway) < .5) {
						InRange++;
					} else {
						InRange = 0;
					}
					
					double currentAngle = (IMU.getAngle());
					double pidOutputPower = pid.getOutput(currentAngle);	            
					rd.tankDrive(pidDriveBasePower + (pidOutputPower/4), pidDriveBasePower + (-pidOutputPower/4));
				} else {
					rd.tankDrive(0, 0);
				}
				break;
			case MIDDLE_GEAR:
				if(currTime > Constants.MIDDLE_GEAR_TIME_LIMIT) {
					rd.tankDrive(0, 0);
					break; //We done!
				}
				
				if((tof.GetDataInMillimeters() <= Constants.GEAR_DISTANCE) && (tof.GetDataInMillimeters() > 0)) {
					if(timeHolder == 0) timeHolder = currTime; //get the time when the motors go into push gear mode.
						
					if(currTime < (Constants.GEAR_PUSH_TIME + timeHolder)) {
						double currentAngle = (IMU.getAngle());
						double pidOutputPower = pid.getOutput(currentAngle); 
						rd.tankDrive(Constants.GEAR_PUSH_POWER + (pidOutputPower/4), Constants.GEAR_PUSH_POWER + (-pidOutputPower/4));
					} else {
						rd.tankDrive(0, 0); //We done!
					}
					
					break;
	        	} else {
	        		double currentAngle = IMU.getAngle();
	            	double pidOutputPower = pid.getOutput(currentAngle);
	            
	            	rd.tankDrive(Constants.GEAR_POWER + (pidOutputPower/4), Constants.GEAR_POWER + (-pidOutputPower/4));
	        	}
				break;
			case LEFT_GEAR:
				if(currTime > Constants.SIDE_GEAR_TIME_LIMIT) {
					rd.tankDrive(0, 0);
					break; //We done!
				}
				
				pid.setSetpoint(60 / (1 + Math.exp(-(Constants.SIDE_GEAR_TURN_RATE * (currTime - Constants.SIDE_GEAR_MID_TURN)))));
				
				if((tof.GetDataInMillimeters() <= Constants.GEAR_DISTANCE) && (tof.GetDataInMillimeters() > 0)) {
					if(timeHolder == 0) timeHolder = currTime; //get the time when the motors go into push gear mode.
						
					if(currTime < (Constants.GEAR_PUSH_TIME + timeHolder)) {
						double currentAngle = (IMU.getAngle());
						double pidOutputPower = pid.getOutput(currentAngle); 
						rd.tankDrive(Constants.GEAR_PUSH_POWER + (pidOutputPower/4), Constants.GEAR_PUSH_POWER + (-pidOutputPower/4));
					} else {
						rd.tankDrive(0, 0); //We done!
					}
					
					break;
	        	} else {
	        		double currentAngle = IMU.getAngle();
	            	double pidOutputPower = pid.getOutput(currentAngle);
	            
	            	rd.tankDrive(Constants.GEAR_POWER + (pidOutputPower/4), Constants.GEAR_POWER + (-pidOutputPower/4));
	        	}
				break;
			case RIGHT_GEAR:
				if(currTime > Constants.SIDE_GEAR_TIME_LIMIT) {
					rd.tankDrive(0, 0);
					break; //We done!
				}
				
				pid.setSetpoint(-60 / (1 + Math.exp(-(Constants.SIDE_GEAR_TURN_RATE * (currTime - Constants.SIDE_GEAR_MID_TURN)))));
				
				if((tof.GetDataInMillimeters() <= Constants.GEAR_DISTANCE) && (tof.GetDataInMillimeters() > 0)) {
					if(timeHolder == 0) timeHolder = currTime; //get the time when the motors go into push gear mode.
						
					if(currTime < (Constants.GEAR_PUSH_TIME + timeHolder)) {
						double currentAngle = (IMU.getAngle());
						double pidOutputPower = pid.getOutput(currentAngle); 
						rd.tankDrive(Constants.GEAR_PUSH_POWER + (pidOutputPower/4), Constants.GEAR_PUSH_POWER + (-pidOutputPower/4));
					} else {
						rd.tankDrive(0, 0); //We done!
					}
					
					break;
	        	} else {
	        		double currentAngle = IMU.getAngle();
	            	double pidOutputPower = pid.getOutput(currentAngle);
	            
	            	rd.tankDrive(Constants.GEAR_POWER + (pidOutputPower/4), Constants.GEAR_POWER + (-pidOutputPower/4));
	        	}
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

		if (gamepad.getRawButton(Constants.MODE0_BUTTON)) { // left top
			mode = 0; // regular
		} else if (gamepad.getRawButton(Constants.MODE1_BUTTON)) { // right top
			mode = 1; // 3/4 speed
		} else if (gamepad.getRawButton(Constants.MODE2_BUTTON)) { // B button
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

		if (operatorJoystick.getRawButton(3) || gamepad.getRawAxis(2) > 0.3) {
			winchMotorA.set(0.4);
		} else if(operatorJoystick.getRawButton(4)) {
			winchMotorA.set(-0.4);
		} else if (operatorJoystick.getRawButton(2)) {
			winchMotorA.set(-operatorJoystick.getY());
		} else {
			winchMotorA.set(0);
		}
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
}