package org.usfirst.frc.team972.robot;

public class Constants {
	// Robot Information
	public static final double ROBOT_WIDTH = 0.751; // in meters
	public static final double ROBOT_LENGTH = 0.737; // in meters
	public static final double LENGTH_GEAR_PEG = 0.1; // in meters distance from the plane which includes the gear vision tape 
	public static final double ROBOT_MAX_VELOCITY = 3.5; // m/s
	public static final double ROBOT_DRIVE_WHEEL_CIRCUMFERENCE = 0.320; // meters (diameter 0.101 meters)
	public static final double ENCODER_CLICKS_PER_ROTATION = 2048;
	public static final double ROBOT_MASS = 36;
	
	// System Model
	public static final double ALPHA = 1.0;
	public static final double BETA = 0.6;
	public static final double PHI = 0.98;
	public static final double SYSVEL = 0.6;
	public static final double SYSACC = 0.75;

	// Auton Drive
	public static final double AUTON_DRIVE_RATIO = 0.7;
	public static final double AUTON_STOPPING_DISTANCE_1 = 1.5; // distance in meters to start stopping (sharp deceleration)
	public static final double AUTON_STOPPING_DISTANCE_2 = 0.5; // distance in meters to finish stopping (lower deceleration) (less than distance 1)
	public static final double AUTON_VELOCITY_STOPPING_PROPORTION = 0.35; // proportion of max velocity city that should be reached after first stopping
	public static final double AUTON_DRIVE_VP = 0.12; // proportion of velocity error
	public static final double AUTON_DRIVE_VD = 0.016; // proportion of acceleration (should be small)
	public static final double AUTON_DRIVE_AP = 0.01; // proportion of angle error (during motion)
	public static final double AUTON_DRIVE_AD = 0.0005; // proportion of angle change (during motion) (should be small)
	public static final double AUTON_DRIVE_F = 0.15; //feed forward
	public static final double AUTON_DRIVE_TURNP = 0.005; // proportion of angle error (during motion)
	public static final double AUTON_DRIVE_TURND = 0.0012; // proportion of angle change (during motion) (should be small)
	public static final double AUTON_DRIVE_TURNF = 0.05; //feed forward

	public static final String LOGGER_LOCATION = "home/admin/FRC2017Logs";
	
	// Middle Gear Auton defaults
	public static final double MIDDLE_GEAR_AUTO_STARTX = 4.112;
	public static final double MIDDLE_GEAR_AUTO_STARTY = 0.502;
	public static final double MIDDLE_GEAR_AUTO_X = 4.112;
	public static final double MIDDLE_GEAR_AUTO_Y = 2.115;
	public static final double MIDDLE_GEAR_AUTO_THETA = 0.0;
	
	// Left Gear Auton defaults
	public static final double LEFT_GEAR_AUTO_STARTX = 1.489;
	public static final double LEFT_GEAR_AUTO_STARTY = 0.502;
	public static final double LEFT_GEAR_AUTO_X = 1.489;
	public static final double LEFT_GEAR_AUTO_Y = 1.774;
	public static final double LEFT_GEAR_AUTO_FINAL_X = 2.521;
	public static final double LEFT_GEAR_AUTO_FINAL_Y = 2.871;
	public static final double LEFT_GEAR_AUTO_THETA = 60.0;
	
	// Right Gear Auton defaults
	public static final double RIGHT_GEAR_AUTO_STARTX = 6.772;
	public static final double RIGHT_GEAR_AUTO_STARTY = 0.502;
	public static final double RIGHT_GEAR_AUTO_X = 6.772;
	public static final double RIGHT_GEAR_AUTO_Y = 1.774;
	public static final double RIGHT_GEAR_AUTO_FINAL_X = 5.702;
	public static final double RIGHT_GEAR_AUTO_FINAL_Y = 2.871;
	public static final double RIGHT_GEAR_AUTO_THETA = -60.0;
	
	public static final double IDEAL_SHOOTING_DISTANCE = 2.0; //TODO determine
}
