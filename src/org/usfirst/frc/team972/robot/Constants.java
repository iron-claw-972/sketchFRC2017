package org.usfirst.frc.team972.robot;

public class Constants {
	public static final double BASELINE_DISTANCE = 10; //feet

	public static final double GEAR_POWER = 0.5; //We drive at this power
	public static final double GEAR_DISTANCE = 100; //Until we get to this distance away using the tof sensor
	public static final double GEAR_PUSH_TIME = 0.5; //Then we push for this long
	public static final double GEAR_PUSH_POWER = 0.25; //With this much power
	public static final double MIDDLE_GEAR_TIME_LIMIT = 5; //And if we go this long, we auto-stop (in case sensor breaks or something)
	public static final double SIDE_GEAR_TIME_LIMIT = 5; //And if we go this long, we auto-stop (in case sensor breaks or something)
	
	public static final double SIDE_GEAR_TURN_RATE = 2.0;
	public static final double SIDE_GEAR_MID_TURN = 2.0;
	
	public static final double TURNP = 0.035;
	public static final double TURNI = 0.0;
	public static final double TURND = 0.0;
	
	public static final double DISTP = 0.012;
	public static final double DISTI = 0.0;
	public static final double DISTD = 0.0;
	public static final double DISTF = 0.15;
	
	public static final double PID_OUTPUT_LIMIT = 0.5;
	public static final double PID_RAMP_RATE = 0.02;
	
	public static final int MODE0_BUTTON = 5;
	public static final int MODE1_BUTTON = 6;
	public static final int MODE2_BUTTON = 2;
}