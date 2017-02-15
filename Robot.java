
package org.usfirst.frc.team5684.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CameraServer;
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
import org.usfirst.frc.team5684.robot.commands.Climb;
import org.usfirst.frc.team5684.robot.commands.DriveStraight;
import org.usfirst.frc.team5684.robot.commands.LowerArm;
import org.usfirst.frc.team5684.robot.commands.Shoot;
import org.usfirst.frc.team5684.robot.commands.StartShooter;
import org.usfirst.frc.team5684.robot.commands.Startup;
import org.usfirst.frc.team5684.robot.commands.TestShooter;
import org.usfirst.frc.team5684.robot.subsystems.Arm;
import org.usfirst.frc.team5684.robot.subsystems.CamMount;
import org.usfirst.frc.team5684.robot.subsystems.CenterPeg;
import org.usfirst.frc.team5684.robot.subsystems.Climber;
import org.usfirst.frc.team5684.robot.subsystems.Collector;
import org.usfirst.frc.team5684.robot.subsystems.DriveTrain;
import org.usfirst.frc.team5684.robot.subsystems.ExampleSubsystem;
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
	public static AnalogGyro myGyro;
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
	public static StartShooter s;

	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		showNomral=true;
		myGyro = new AnalogGyro(RobotMap.gyro);
		
		myGyro.reset();
		myGyro.setSensitivity(voltsPerDegreePerSecond);
		myGyro.calibrate();
		turn = new Turn();
		climber = new Climber();
		collector = new Collector();
		shooter = new Shooter();
		camMount = new CamMount();
		centerPeg = new CenterPeg();
		oi = new OI();
		
		chooser.addDefault("Test Shooter",new TestShooter());
		chooser.addObject("Drive Forward", new DriveStraight());
		//chooser.addObject("Climb", new Climb());
		
		
		SmartDashboard.putData("Auto mode", chooser);
		
		/*
		 * camera error code
		 * ERROR: ioctl VIDIOC_S_CTRL failed at UsbCameraProperty.cpp:72: Input/output error (UsbUtil.cpp:122) 
		 * ERROR: ioctl VIDIOC_S_CTRL failed at UsbCameraProperty.cpp:72: Input/output error (UsbUtil.cpp:122) 

		 */
		new Thread(() -> {
			boolean showBad = false;
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			camera.setResolution(640, 480);
			camera.setExposureManual(-100);
			camera.setBrightness(15);
			CvSink cvSink = CameraServer.getInstance().getVideo();
			CvSource outputStream = CameraServer.getInstance().putVideo("Otto", IMG_WIDTH, IMG_HEIGHT);
			//if(!showNomral)
			//{
				
				
			Mat source = new Mat();
			Mat output = new Mat();
			double[] red = { 0, 255 };
			double[] green = { 20, 255 };
			double[] blue = { 0, 255 };
			ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
			boolean externalOnly = true;
			Mat hierarchy = new Mat();
			MatOfInt hull = new MatOfInt();
			convexHullsOutput = new ArrayList<MatOfPoint>();
			while (!Thread.interrupted()) {
				cvSink.grabFrame(source);

				Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2RGB);

				Core.inRange(source, new Scalar(red[0], green[0], blue[0]), new Scalar(red[1], green[1], blue[1]),
						output);

				int mode;
				if (externalOnly) {
					mode = Imgproc.RETR_EXTERNAL;
				} else {
					mode = Imgproc.RETR_LIST;
				}
				int method = Imgproc.CHAIN_APPROX_SIMPLE;
				findContoursOutput.clear();
				Imgproc.findContours(output, findContoursOutput, hierarchy, mode, method);

				convexHullsOutput.clear();
				for (int i = 0; i < findContoursOutput.size(); i++) {
					final MatOfPoint contour = findContoursOutput.get(i);
					final MatOfPoint mopHull = new MatOfPoint();
					Imgproc.convexHull(contour, hull);
					mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
					for (int j = 0; j < hull.size().height; j++) {
						int index = (int) hull.get(j, 0)[0];
						double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1] };
						mopHull.put(j, 0, point);
					}
					convexHullsOutput.add(mopHull);
				}
				int maxX = Integer.MIN_VALUE;
				int maxY = Integer.MIN_VALUE;

				int minX = Integer.MAX_VALUE;
				int minY = Integer.MAX_VALUE;
				for (int i = 0; i < convexHullsOutput.size(); i++) {
					int tempMinX = Integer.MAX_VALUE;
					int tempMinY = Integer.MAX_VALUE;
					int tempMaxX = Integer.MIN_VALUE;
					int tempMaxY = Integer.MIN_VALUE;
					Point[] pointsTemp = convexHullsOutput.get(i).toArray();
					if (isGood(pointsTemp, 5.0 / 2.0, .66, 50)) {
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

							// create the recatangle for just this shape

							if (pointsTemp[q].x > tempMaxX) {
								tempMaxX = (int) pointsTemp[q].x;
							}
							if (pointsTemp[q].x < tempMinX) {
								tempMinX = (int) pointsTemp[q].x;
							}
							if (pointsTemp[q].y > tempMaxY) {
								tempMaxY = (int) pointsTemp[q].y;
							}
							if (pointsTemp[q].x < tempMinY) {
								tempMinY = (int) pointsTemp[q].y;
							}
						}
						Imgproc.rectangle(source, new Point(tempMinX, tempMinY), new Point(tempMaxX, tempMaxY),
								new Scalar(0, 255, 0), 5);
						Imgproc.putText(source,
								"" + new DecimalFormat("#.##").format(getSize(pointsTemp)) + ","
										+ new DecimalFormat("#.##").format(getRatio(pointsTemp)),
								new Point(tempMinX, tempMinY), Core.FONT_HERSHEY_COMPLEX, .5, new Scalar(0, 255, 0));

						double targetHeight = 5;
						double targetWidth=2;
						double pixelHeight = (tempMaxY - tempMinY);
						double pixelWidth = (tempMaxX - tempMinX);
						double theta = .405458; //theta is in radians;
						
						double distanceH = (targetHeight*IMG_HEIGHT)/(2*pixelHeight*Math.tan(theta));
						double distanceW = (targetWidth*IMG_WIDTH)/(2*pixelWidth*Math.tan(theta));
						String distanceString = "(5 * ("+(tempMaxY - tempMinY)+")) / (2 * ("+(tempMaxX - tempMinX)+") * Math.tan(Math.toRadians(68.5/2)))";
						Imgproc.putText(source, new DecimalFormat("#.##").format(distanceW)+"",
								new Point(tempMinX, tempMaxY + 15), Core.FONT_HERSHEY_COMPLEX, .5,
								new Scalar(0, 255, 0));
						Imgproc.putText(source, new DecimalFormat("#.##").format(distanceW),
								new Point(tempMinX, tempMaxY + 15), Core.FONT_HERSHEY_COMPLEX, .5,
								new Scalar(0, 255, 0));
						
					} else if (showBad) {
						for (int q = 0; q < pointsTemp.length; q++) {
							if (pointsTemp[q].x > tempMaxX) {
								tempMaxX = (int) pointsTemp[q].x;
							}
							if (pointsTemp[q].x < tempMinX) {
								tempMinX = (int) pointsTemp[q].x;
							}
							if (pointsTemp[q].y > tempMaxY) {
								tempMaxY = (int) pointsTemp[q].y;
							}
							if (pointsTemp[q].x < tempMinY) {
								tempMinY = (int) pointsTemp[q].y;
							}
							Imgproc.rectangle(source, new Point(tempMinX, tempMinY), new Point(tempMaxX, tempMaxY),
									new Scalar(0, 0, 255), 5);
						}
					}
				}

				// error correcting. find the center of each convex hull, then
				// find the midpoint of the two centers.

				int middleX = (maxX + minX) / 2;
				int middleY = (maxY + minY) / 2;

				pegX = middleX;
				pegY = middleY;
				SmartDashboard.putNumber("PegX", pegX);
				SmartDashboard.putNumber("pegY", pegY);

				int minRecX = middleX - 5;
				int maxRecX = middleX + 5;

				int minRecY = middleY - 5;
				int maxRecY = middleY + 5;
				sizeOfTarget = maxRecX - minRecX;

				// Put a rectangle on the image
				Imgproc.rectangle(source, new Point(minRecX, minRecY), new Point(maxRecX, maxRecY),
						new Scalar(255, 0, 0), 5);
				// Give the output stream a new image to display
			//}
				outputStream.putFrame(source);

			}
		}).start();
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
		System.out.println("chooser: "+ chooser);
		System.out.println("command: " + chooser.getSelected());
		s = new StartShooter();
		s.initialize();
		s.execute();
		autonomousCommand = chooser.getSelected();
		System.out.println("command: " + autonomousCommand);
		
		
		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (autonomousCommand != null)
		{
			System.out.println("\t We got inside");
			
		}
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		if(s.isFinished())
		{
			s.end();
		}
		
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
		
	}

	@Override
	public void teleopInit() {
		Startup start = new Startup();
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null)
			autonomousCommand.cancel();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
	
	public void robotPeriodic(){
		
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
