
package org.usfirst.frc.team5684.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Arrays;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team5684.robot.commands.Agitate;
import org.usfirst.frc.team5684.robot.commands.AutoForward;
import org.usfirst.frc.team5684.robot.commands.AutoPlacePegAndBlue;
import org.usfirst.frc.team5684.robot.commands.AutoPlacePegAndRed;
import org.usfirst.frc.team5684.robot.commands.AutoPlacePegAndStop;
import org.usfirst.frc.team5684.robot.commands.Climb;
import org.usfirst.frc.team5684.robot.commands.Collect;
import org.usfirst.frc.team5684.robot.commands.DriveStraight;
import org.usfirst.frc.team5684.robot.commands.LowerArm;
import org.usfirst.frc.team5684.robot.commands.RaiseArm;
import org.usfirst.frc.team5684.robot.commands.Shoot;
import org.usfirst.frc.team5684.robot.commands.Square;
import org.usfirst.frc.team5684.robot.commands.Startup;
import org.usfirst.frc.team5684.robot.subsystems.Agitator;
import org.usfirst.frc.team5684.robot.subsystems.Arm;
import org.usfirst.frc.team5684.robot.subsystems.CamMount;
import org.usfirst.frc.team5684.robot.subsystems.CenterPeg;
import org.usfirst.frc.team5684.robot.subsystems.Climber;
import org.usfirst.frc.team5684.robot.subsystems.Collector;
import org.usfirst.frc.team5684.robot.subsystems.DriveStraightBackwardsPID;
import org.usfirst.frc.team5684.robot.subsystems.DriveStraightPID;
import org.usfirst.frc.team5684.robot.subsystems.DriveTrain;
import org.usfirst.frc.team5684.robot.subsystems.Shooter;
import org.usfirst.frc.team5684.robot.subsystems.Turn;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	public static final Arm arm = new Arm();
	public static OI oi;
	public static DriveTrain drivetrain = new DriveTrain();
	public static Ultrasonic myProxSensor; // SCOTT HOWARD - declaring the
											// sensor
	public static double voltsPerDegreePerSecond = 0.0012;
	public static final int IMG_WIDTH = 640;
	public static final int IMG_HEIGHT = 480;
	public static int pegX;
	public static int pegY;
	public static double centerX = 0.00;
	public static double sizeOfTarget = 0.00;
	public static Turn turn;
	public static ArrayList<MatOfPoint> convexHullsOutput;
	public static Climber climber;
	public static Collector collector;
	public static Shooter shooter;
	public static CamMount camMount;
	public static CenterPeg centerPeg;
	public static boolean showNomral;
	public static Agitator agitator;
	public static ADIS16448_IMU imu;
	public static DriveStraightPID driveStraightPID;
	public static DriveStraightBackwardsPID driveStraightBackwardsPID;
	public static Square square;
	public static Encoder enc;
	Thread visionThread;
	public long startTime;
	public long endTime;
	public static int temp = 1;

	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		showNomral = true;

		// SCOTT HOWARD - parameters will need to be adjusted. Parameters are
		// (echo pulse, trigger pulse)
		// So, this example uses DigitalOutput 0 as the echo pulse, and
		// DigitalInput 0 for the trigger pulse
		// I'm not sure what specific ports you'll be using, so I put 0's in for
		// place-holders
		// myProxSensor = new Ultrasonic(0, 0);
		// SCOTT HOWARD - this function call simply reads the range on the
		// sensor. Maybe print this to the screen?
		// According to FRC, accuracy should be within 2-3 inches. Just not sure
		// on specific numbers for the Pololu sensor.
		// double range = myProxSensor.getRangeInches();
		turn = new Turn();
		climber = new Climber();
		collector = new Collector();
		shooter = new Shooter();
		camMount = new CamMount();
		centerPeg = new CenterPeg();
		agitator = new Agitator();
		imu = new ADIS16448_IMU();
		driveStraightPID = new DriveStraightPID();
		driveStraightBackwardsPID = new DriveStraightBackwardsPID();
		oi = new OI();

		SmartDashboard.putData(turn);
		SmartDashboard.putData(climber);
		SmartDashboard.putData(collector);
		SmartDashboard.putData(shooter);
		SmartDashboard.putData(agitator);
		SmartDashboard.putData(driveStraightPID);

		chooser.addDefault("Place peg and stop", new AutoPlacePegAndStop());
		chooser.addObject("Startup", new Startup());
		chooser.addObject("Drive Forward", new AutoForward());
		chooser.addObject("Peg and Shoot Red", new AutoPlacePegAndRed());
		chooser.addObject("Peg and Shoot Blue", new AutoPlacePegAndBlue());
		chooser.addObject("Square", new Square());

		SmartDashboard.putData("Auto mode", chooser);

		SmartDashboard.putNumber(RobotMap.agitatorSlider, 2.5);
		SmartDashboard.putNumber(RobotMap.shooterSlider, 3.0);
		SmartDashboard.putNumber(RobotMap.collectorSlider, 2.5);
		SmartDashboard.putNumber(RobotMap.climbSlider, 2.5);

		/*
		 * camera error code ERROR: ioctl VIDIOC_S_CTRL failed at
		 * UsbCameraProperty.cpp:72: Input/output error (UsbUtil.cpp:122) ERROR:
		 * ioctl VIDIOC_S_CTRL failed at UsbCameraProperty.cpp:72: Input/output
		 * error (UsbUtil.cpp:122)
		 * 
		 */
		visionThread = new Thread(() -> {
			// Get the UsbCamera from CameraServer
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			// Set the resolution
			camera.setResolution(640, 480);

			// Get a CvSink. This will capture Mats from the camera
			CvSink cvSink = CameraServer.getInstance().getVideo();
			// Setup a CvSource. This will send images back to the Dashboard
			CvSource outputStream = CameraServer.getInstance().putVideo("Rectangle", 640, 480);

			// Mats are very memory expensive. Lets reuse this Mat.
			Mat mat = new Mat();

			// This cannot be 'true'. The program will never exit if it is. This
			// lets the robot stop this thread when restarting robot code or
			// deploying.
			while (!Thread.interrupted()) {
				// Tell the CvSink to grab a frame from the camera and put it
				// in the source mat. If there is an error notify the output.
				if (cvSink.grabFrame(mat) == 0) {
					// Send the output the error.
					outputStream.notifyError(cvSink.getError());
					// skip the rest of the current iteration
					continue;
				}
				// Put a rectangle on the image
				// Give the output stream a new image to display
				if (((endTime - System.currentTimeMillis()) / 1000.0) < 30) {
					if (temp == 1) {
						Imgproc.putText(mat, "" + (endTime - System.currentTimeMillis()) / 1000.0 + " secs",
								new Point(50, 50), Core.FONT_HERSHEY_COMPLEX, 1, new Scalar(0, 0, 0));
					} else {
						Imgproc.putText(mat, "" + (endTime - System.currentTimeMillis()) / 1000.0 + " secs",
								new Point(50, 50), Core.FONT_HERSHEY_COMPLEX, 1, new Scalar(255, 255, 255));
					}
					temp = -1 * temp + 1;
				}
				outputStream.putFrame(mat);
			}
		});
		visionThread.setDaemon(true);
		visionThread.start();
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		startTime = System.currentTimeMillis();
		endTime = startTime + 1000 * 15;
		autonomousCommand = chooser.getSelected();

		/*
		 * SCOTT HOWARD int command = 0; switch(command) {
		 * 
		 * // Switch on different integers for each command (StartShooter,
		 * Collect, etc...) // Up to you guys on what case you want to be which
		 * command // This might be where you can set the 4 bits for each option
		 * (1000, 0001, 1101, etc...)
		 * 
		 * }
		 */

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (autonomousCommand != null) {
			autonomousCommand.start();

		}
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();

	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		startTime = System.currentTimeMillis();
		endTime = startTime + 1000 * 135;
		if (autonomousCommand != null)
			autonomousCommand.cancel();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		SmartDashboard.putNumber("Angle Headding", imu.getAngleX());
		SmartDashboard.putNumber("X Acc", imu.getAccelX());
		SmartDashboard.putNumber("Y Acc", imu.getAccelY());
		SmartDashboard.putNumber("Z Acc", imu.getAccelZ());
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}

	public void robotPeriodic() {

	}

	public int getMax(ArrayList<Point> listOfPoints, int z) {

		int max = Integer.MIN_VALUE;
		for (int i = 0; i < listOfPoints.size(); i++) {
			int temp = 0;
			if (z == 0) {
				temp = (int) listOfPoints.get(i).x;
			} else {
				temp = (int) listOfPoints.get(i).y;
			}
			if (temp > max)
				max = temp;
		}
		return max;
	}

	public int getMin(ArrayList<Point> listOfPoints, int z) {

		int min = Integer.MAX_VALUE;
		for (int i = 0; i < listOfPoints.size(); i++) {
			int temp = 0;
			if (z == 0) {
				temp = (int) listOfPoints.get(i).x;
			} else {
				temp = (int) listOfPoints.get(i).y;
			}
			if (temp < min)
				min = temp;
		}
		return min;
	}

	public int getMin(Point[] listOfPoints, int z) {

		int min = Integer.MAX_VALUE;
		for (int i = 0; i < listOfPoints.length; i++) {
			int temp = 0;
			if (z == 0) {
				temp = (int) listOfPoints[i].x;
			} else {
				temp = (int) listOfPoints[i].y;
			}
			if (temp < min)
				min = temp;
		}
		return min;
	}

	public int getMax(Point[] listOfPoints, int z) {

		int max = Integer.MIN_VALUE;
		for (int i = 0; i < listOfPoints.length; i++) {
			int temp = 0;
			if (z == 0) {
				temp = (int) listOfPoints[i].x;
			} else {
				temp = (int) listOfPoints[i].y;
			}
			if (temp > max)
				max = temp;
		}
		return max;
	}

	public double getRatio(Point[] pointsTemp) {
		int maxX = Integer.MIN_VALUE;
		int maxY = Integer.MIN_VALUE;

		int minX = Integer.MAX_VALUE;
		int minY = Integer.MAX_VALUE;

		double ratio = 0;

		for (int q = 0; q < pointsTemp.length; q++) {
			if (pointsTemp[q].x > maxX) {
				maxX = (int) pointsTemp[q].x;
			}
			if (pointsTemp[q].x < minX) {
				minX = (int) pointsTemp[q].x;
			}
			if (pointsTemp[q].y > maxY) {
				maxY = (int) pointsTemp[q].y;
			}
			if (pointsTemp[q].x < minY) {
				minY = (int) pointsTemp[q].y;
			}

		}
		return (maxY - minY) / (maxX - minX * 1.0);
	}

	public boolean isGood(Point[] pointsTemp, double goalRatio, double errorRate, double minSize) {
		int maxX = Integer.MIN_VALUE;
		int maxY = Integer.MIN_VALUE;

		int minX = Integer.MAX_VALUE;
		int minY = Integer.MAX_VALUE;

		for (int q = 0; q < pointsTemp.length; q++) {
			if (pointsTemp[q].x > maxX) {
				maxX = (int) pointsTemp[q].x;
			}
			if (pointsTemp[q].x < minX) {
				minX = (int) pointsTemp[q].x;
			}
			if (pointsTemp[q].y > maxY) {
				maxY = (int) pointsTemp[q].y;
			}
			if (pointsTemp[q].x < minY) {
				minY = (int) pointsTemp[q].y;
			}
		}
		double ratio = (maxY - minY) / (maxX - minX * 1.0);
		double size = (maxY - minY) * (maxX - minX * 1.0);

		return (Math.abs(ratio - goalRatio) < errorRate && size > minSize);
	}

	public double getSize(Point[] pointsTemp) {
		int maxX = Integer.MIN_VALUE;
		int maxY = Integer.MIN_VALUE;

		int minX = Integer.MAX_VALUE;
		int minY = Integer.MAX_VALUE;

		double ratio = 0;

		for (int q = 0; q < pointsTemp.length; q++) {
			if (pointsTemp[q].x > maxX) {
				maxX = (int) pointsTemp[q].x;
			}
			if (pointsTemp[q].x < minX) {
				minX = (int) pointsTemp[q].x;
			}
			if (pointsTemp[q].y > maxY) {
				maxY = (int) pointsTemp[q].y;
			}
			if (pointsTemp[q].x < minY) {
				minY = (int) pointsTemp[q].y;
			}
		}

		return (maxY - minY) * (maxX - minX * 1.0);
	}
}
