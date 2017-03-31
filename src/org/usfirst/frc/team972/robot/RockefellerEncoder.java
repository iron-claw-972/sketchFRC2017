package org.usfirst.frc.team972.robot;

import edu.wpi.first.wpilibj.Encoder;

public class RockefellerEncoder {
	Encoder internalEncoder;
	
	private double ENCODER_CLICKS_PER_ROTATION = 2048;
	private double ROBOT_DRIVE_WHEEL_CIRCUMFERENCE = 12.6 / 12; //feet
	
	public RockefellerEncoder(int port1, int port2, boolean setReversed) {
		internalEncoder = new Encoder(port1, port2, setReversed, Encoder.EncodingType.k2X);
	}
	
	public void reset() {
		internalEncoder.reset();
	}
	
	public int getClicks() {
		return internalEncoder.get();
	}
	
	public double getDistance() {
		return internalEncoder.get() * ROBOT_DRIVE_WHEEL_CIRCUMFERENCE / ENCODER_CLICKS_PER_ROTATION;
	}
	
	public double getRotations() {
		return internalEncoder.get() / ENCODER_CLICKS_PER_ROTATION;
	}
}