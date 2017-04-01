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
	
	boolean run = false;
	
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

	DoubleSolenoid piston = new DoubleSolenoid(30, 0, 2);

	Joystick gamepad = new Joystick(1);
	Joystick operatorJoystick = new Joystick(0);
	RobotDrive rd = new RobotDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
	
	boolean winchPressedLastTime = false;
	boolean runWinch = false;

	boolean backMode = false;
	boolean backPressedLastTime = false;
	
	double leftSpeed = 0.0;
	double rightSpeed = 0.0;

	long startTime = 0;

	final String BASELINE = "Baseline";
	final String MIDDLE_GEAR = "Middle Gear";
	final String LEFT_GEAR = "Left Gear";
	final String RIGHT_GEAR = "Right Gear";
	final String DO_NOTHING = "Do Nothing";
	final String JODY = "JODY";
	String autoSelected;

	SendableChooser<String> autoChooser = new SendableChooser<>();
	
	PIDControl pid;
	PIDControl driveDistancePid;
	
	@Override
	public void robotInit() {		
		IMU.init();
		autoChooser.addDefault("JODY", JODY);
		autoChooser.addObject("Baseline", BASELINE);
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
		//IMU.init();
		startTime = System.currentTimeMillis();
		
		run = true;
		
		/*
		try {
			if(tof != null) {
  			tof.port.closePort();
			}
	    	tof = new TimeOfFlight();
		} catch(Exception e) {
			e.printStackTrace();
			System.out.println("TIME OF FLIGHT SENSOR COULD NOT WORK FOR SOME REASON.");
		} */
		
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
				while (System.currentTimeMillis() < startTime + 1323) {
					rd.tankDrive(0.6, 0.6);
				}
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
			case JODY:
		        pid = new PIDControl(Constants.TURNP, Constants.TURNI, Constants.TURND);
		        pid.setOutputLimits(Constants.PID_OUTPUT_LIMIT);
		        pid.setSetpoint(0);
		        
		        driveDistancePid = new PIDControl(Constants.DISTP, Constants.DISTI, Constants.DISTD, Constants.DISTF);
		        driveDistancePid.setOutputLimits(Constants.PID_OUTPUT_LIMIT);
		        driveDistancePid.setSetpoint(0);
		        driveDistancePid.setOutputRampRate(Constants.PID_RAMP_RATE);
		        
		        System.out.println("RUNNING JODY CODE");
		        
				//cool code
				
		        //DriveStraightEncoder(4, -0.6, 6); // distance is negative because encoders flipped :/
				//DriveStraight(0.5, -0.5);
				
		        DriveStraightEncoder(4, -0.6, 4.88);
		        waitThread(1000);
				DriveTurn(60, 5);
				waitThread(1000);
//				DriveStraightEncoder(2, -0.6, 3.2);
				
				
				//GHETTO BECAUSE NO IMU RESET...
				rightDriveEncoderBack.reset();
		        pid.reset();
		        
		        System.out.println("Starting Encoder Drive: " + rightDriveEncoderBack.getDistance());
		        
		        for(int i=0; i<50 * 3; i++) {
		        	if(Math.abs(rightDriveEncoderBack.getDistance()) > 3.4) {
		        		System.out.println("Meet dist: " + 3.4);
		        		break;
		        	}
		            double currentAngle = IMU.getAngle() - 60;
		            double pidOutputPower = pid.getOutput(currentAngle);
		            //System.out.println(leftDriveEncoderFront.getDistance() + " " + leftDriveEncoderBack.getDistance() + " " + rightDriveEncoderFront.getDistance() + " " + rightDriveEncoderBack.getDistance());
		            System.out.println(rightDriveEncoderBack.getDistance());
		            rd.tankDrive(-0.6 + (pidOutputPower/4), -0.6 + (pidOutputPower/4));
		            waitThread(20);
		        }
		        rd.tankDrive(0, 0);
				
				
				//turn 45 for test
				
			break;
		}
	}

	int InRange = 0;
	
	/*
	
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
	} */

	public void teleopInit() {
		CameraServer.getInstance().startAutomaticCapture();

		run = false;
		
        pid = new PIDControl(Constants.TURNP, Constants.TURNI, Constants.TURND);
        pid.setOutputLimits(Constants.PID_OUTPUT_LIMIT);
        pid.setSetpoint(0);
        
        IMU.recalibrate(0.0);
        
		try {
			new Compressor(30).start();
		} catch (Exception e) {
			e.printStackTrace();
			System.out.println("COMPRESSOR FAILED!");
		}

		piston.set(DoubleSolenoid.Value.kReverse);
		
		leftDriveEncoderFront.reset();
		rightDriveEncoderFront.reset();
		leftDriveEncoderBack.reset();
		rightDriveEncoderBack.reset();

	}
	
	public void disabledPeriodic() {
		piston.set(DoubleSolenoid.Value.kReverse);
		run = false;
	}

	@Override
	public void teleopPeriodic() {
		
		SmartDashboard.putNumber("Left Encoder Front", leftDriveEncoderFront.getClicks());
		SmartDashboard.putNumber("Left Encoder Back", leftDriveEncoderBack.getClicks());
		SmartDashboard.putNumber("Right Encoder Front", rightDriveEncoderFront.getClicks());
		SmartDashboard.putNumber("Right Encoder Back", rightDriveEncoderBack.getClicks());
		
		if (operatorJoystick.getRawButton(1) || gamepad.getRawButton(3)) {// joy trigger or gamepad X button
			piston.set(DoubleSolenoid.Value.kForward);// forward is up, reverse is ready to receive gears
		} else {
			piston.set(DoubleSolenoid.Value.kReverse);
		}
		
		leftSpeed = gamepad.getRawAxis(5);
		rightSpeed = gamepad.getRawAxis(1);
		
		//System.out.println(leftDriveEncoderFront.get() + " " + rightDriveEncoderFront.get());
		if(gamepad.getRawButton(1)) {
			double pidOutputPower = pid.getOutput(IMU.getAngle());
			
        	if(newHeadingClick) {
        		IMU.recalibrate(0.0);
        		pid.reset();
        		newHeadingClick = false;
        	}
        	System.out.println(pidOutputPower);
            rd.tankDrive(rightSpeed + (pidOutputPower * .25), rightSpeed + (-pidOutputPower * .25));
		} else if(gamepad.getRawButton(2)) {
			double pidOutputPower = pid.getOutput(IMU.getAngle());
			
        	if(newHeadingClick) {
        		IMU.recalibrate(0.0);
        		pid.reset();
        		newHeadingClick = false;
        	}
        	System.out.println(pidOutputPower * leftSpeed);
            rd.tankDrive((pidOutputPower * leftSpeed), (-pidOutputPower * leftSpeed));
		} else {
			rd.tankDrive(leftSpeed, rightSpeed);
			newHeadingClick = true;
		}

		if (operatorJoystick.getRawButton(7) || gamepad.getRawAxis(2) > 0.3) {
			winchMotorA.set(0.9);
		} else if(operatorJoystick.getRawButton(8)) {
			winchMotorA.set(-0.3);
		} else if (operatorJoystick.getRawButton(2)) {

			winchMotorA.set(-operatorJoystick.getY());
		} else {
			winchMotorA.set(0);
		}
		
		//SmartDashboard.putNumber("Distance from Wall", tof.GetDataInMillimeters() * 0.03937); //distance in inches
	}
	
    public double LogisticsCurve(double t, double setPoint, double rate, double midTurn) {
		return setPoint / (1 + Math.exp(-(rate * (t - midTurn))));
    }
    
    public void DriveStraight(double time, double power) {
        IMU.recalibrate(0);
        pid.reset();

        for(int i=0; i<50 * time; i++) {
            double currentAngle = IMU.getAngle();
            double pidOutputPower = pid.getOutput(currentAngle);
            
            rd.tankDrive(power + (pidOutputPower/4), power + (pidOutputPower/4));
            System.out.println(pidOutputPower);
            waitThread(20);
        }
        rd.tankDrive(0, 0);
    }
    
    public void DriveStraightEncoder(double time, double power, double dist) {
    	rightDriveEncoderBack.reset();
        IMU.recalibrate(0);
        pid.reset();
        
        System.out.println("Starting Encoder Drive: " + rightDriveEncoderBack.getDistance());
        
        for(int i=0; i<50 * time; i++) {
        	if(Math.abs(rightDriveEncoderBack.getDistance()) > dist) {
        		System.out.println("Meet dist: " + dist);
        		break;
        	}
            double currentAngle = IMU.getAngle();
            double pidOutputPower = pid.getOutput(currentAngle);
            //System.out.println(leftDriveEncoderFront.getDistance() + " " + leftDriveEncoderBack.getDistance() + " " + rightDriveEncoderFront.getDistance() + " " + rightDriveEncoderBack.getDistance());
            System.out.println(rightDriveEncoderBack.getDistance());
            rd.tankDrive(power + (pidOutputPower/4), power + (pidOutputPower/4));
            waitThread(20);
        }
        rd.tankDrive(0, 0);
    }
    
    public void DriveTurn(double angle, double maxTime) {
    	double maxAccum = 0;
    	pid.reset();
    	IMU.recalibrate(0);

    	pid.setOutputLimits(0.6);
    	
    	//pid.setOutputRampRate(0.5);
    	pid.setP(0.03);
    	pid.setD(0.0);
    	
    	int inTheZone = 0;
    	
        double currentAngle = IMU.getAngle() + angle;
        double pidOutputPower = pid.getOutput(currentAngle);
        
        //System.out.println(pidOutputPower);
        
        System.out.println("Turn Starting");
        
        while((inTheZone < 50)) {
        	maxAccum++;
        	if(maxAccum > (maxTime * 50)) {
        		break;
        	}
        	if(run == false) break;
        	
        	currentAngle = IMU.getAngle() + angle;
        	pidOutputPower = pid.getOutput(currentAngle);
        	
        	System.out.println(currentAngle + " : " + pidOutputPower);
        	
        	if(Math.abs(currentAngle) < 20) {
        		pid.setOutputLimits(0.5);
        		pid.setI(0.025);
        		pid.setMaxIOutput(0.35);
        		inTheZone++;
        	}
        	
            rd.tankDrive(pidOutputPower, -pidOutputPower);
            waitThread(20);
        }
        
        rd.tankDrive(0, 0);
        
        //reset the thing
        pid.setP(Constants.TURNP);
        pid.setOutputRampRate(0);
        pid.setF(0);
        pid.setD(Constants.TURND);
        pid.setI(Constants.TURNI);
    }
    
    public void waitThread(double time) {
        try {
            Thread.sleep((long) time);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
    
	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
}