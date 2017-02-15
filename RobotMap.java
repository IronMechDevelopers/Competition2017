package org.usfirst.frc.team5684.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
	public static int leftWheels=0;
	public static int rightWheels=1;
	public static int arm=6;
	public static int gyro=0;
	public static int climber=4;
	public static int shooter=2;
	public static int collector=3;
	public static int cameraBottom=9;
	public static int cameraTop=8;
	public static String shooterSlider ="DB/Slider 0";
	public static String collectorSlider="DB/Slider 1";
	public static String climbSlider = "DB/Slider 2";
}

