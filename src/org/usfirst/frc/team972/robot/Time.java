package org.usfirst.frc.team972.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Time {
	private static long initTimeMillis = 0;
	
	/**
	 * Sets start time.
	 * 
	 * @see get()
	 */
	public static void init() {
		initTimeMillis = System.currentTimeMillis();
	}
	
	/**
	 * @see init()
	 * @return Time in seconds since init() was called
	 */
	public static double get() {
		return ((double) (System.currentTimeMillis() - initTimeMillis) / 1000.0);
	}
	
	/**
	 * Updates SmartDashboard values for Time.
	 */
	public static void updateSmartDashboard() {
		SmartDashboard.putNumber("Init Time (millis)", initTimeMillis);
		SmartDashboard.putNumber("Current Time", get());
	}
}