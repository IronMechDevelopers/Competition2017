package org.usfirst.frc.team5684.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;

import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team5684.robot.commands.DriveForward;

//import com.ctre.CANTalon;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.vision.VisionRunner;
import edu.wpi.first.wpilibj.vision.VisionThread;

import java.util.ArrayList;
import java.util.Arrays;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();
	Joystick stick = new Joystick(0);
	private static final int IMG_WIDTH = 640;
	private static final int IMG_HEIGHT = 480;
	private double centerX=0.00;
	private double sizeOfTarget=0.00;
	RobotDrive drive = new RobotDrive(0,1);
	//public static DriveTrain drivetrain = new DriveTrain();
	//RobotDrive drive = new RobotDrive(1,2);
	boolean shooterDrive = false; // SCOTT
	private AnalogGyro myGyro=  new AnalogGyro(0);
	//final double voltsPerDegreePerSecond = 0.0125; //trying out
	double voltsPerDegreePerSecond = 0.0012; //original code
	//double voltsPerDegreePerSecond = 0.0001;
	//private double Kp = 0.03;
	public static double SHOOTERSPEED;
	VictorSP shooter=new VictorSP(2);
	int pegX;
	int pegY;
	public SendableChooser autoChooser;
	public SendableChooser autonomousDirectionChooser;
	

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("init");
		/*sampleEncoder.setMaxPeriod(.1);
		sampleEncoder.setMinRate(10);
		sampleEncoder.setDistancePerPulse(5);
		sampleEncoder.setReverseDirection(true);
		sampleEncoder.setSamplesToAverage(7);*/
		
		
		myGyro.reset();
		myGyro.setSensitivity(voltsPerDegreePerSecond);
		myGyro.calibrate();
		
		autoChooser = new SendableChooser();
		autoChooser.addDefault("Gear Left", new DriveForward());
		autoChooser.addObject("Gear Center", new DriveForward());
		autoChooser.addObject("Gear Right", new DriveForward());
		autoChooser.addObject("Shoot low", new DriveForward());
		autoChooser.addObject("shoot high", new DriveForward());
		SmartDashboard.putData("Auto Mode", autoChooser);
		
		new Thread(() -> {
            UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
            camera.setResolution(640, 480);
            camera.setExposureManual(-100);
            camera.setBrightness(15);
            CvSink cvSink = CameraServer.getInstance().getVideo();
            CvSource outputStream = CameraServer.getInstance().putVideo("Radaszkiewicz", IMG_WIDTH, IMG_HEIGHT);
            
            
            Mat source = new Mat();
            Mat output = new Mat();
            double[] red = {0,255};
    		double[] green = {40,255};
    		double[] blue = {0,255};
    		ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
    		boolean externalOnly=false;
    		Mat hierarchy = new Mat();
    		MatOfInt hull = new MatOfInt();
    		ArrayList<MatOfPoint> convexHullsOutput = new ArrayList<MatOfPoint>();
    		ArrayList<Point> points = new ArrayList<Point>();
            while(!Thread.interrupted()) {
                cvSink.grabFrame(source);
                
                Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2RGB);
                
        		Core.inRange(source, new Scalar(red[0], green[0], blue[0]),
        		new Scalar(red[1], green[1], blue[1]), output);
                
        		int mode;
        		if (externalOnly) {
        			mode = Imgproc.RETR_EXTERNAL;
        		}
        		else {
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
        						double[] point = new double[] {contour.get(index, 0)[0], contour.get(index, 0)[1]};
        						mopHull.put(j, 0, point);
        					}
        					convexHullsOutput.add(mopHull);
        				}
        				points.clear();
        				for(int i=0;i<convexHullsOutput.size();i++)
        				{
        					Point[] pointsTemp = convexHullsOutput.get(i).toArray();
        					//System.out.println("Ratio is: " + getRatio(pointsTemp));
        					for(int q=0;q<pointsTemp.length;q++)
        					{
        						points.add(pointsTemp[q]);
        					}
        					int maxXTemp = getMax(points,0);
            				int maxYTemp = getMax(points,1);
            				int minXTemp = getMin(points,0);
            				int minYTemp = getMin(points,1);
        					Imgproc.rectangle(source, new Point(minXTemp, minYTemp), new Point(maxXTemp, maxYTemp),
            						new Scalar(0, 0, 255), 5);
        				}
        				
        				//error correcting.  find the center of each convex hull, then find the midpoint of the two centers.
        				int maxX = getMax(points,0);
        				int maxY = getMax(points,1);
        				int minX = getMin(points,0);
        				int minY = getMin(points,1);
        				
        				int middleX = (maxX+minX)/2;
        				int middleY = (maxY+minY)/2;
        				
        				pegX=middleX;
        				pegY=middleY;
        				SmartDashboard.putNumber("PegX",pegX);
        				SmartDashboard.putNumber("pegY", pegY);
        				
        				int minRecX = middleX-5;
        				int maxRecX = middleX+5;
        				
        				int minRecY = middleY-5;
        				int maxRecY = middleY+5;
        				sizeOfTarget=maxRecX-minRecX;
        				
        				// Put a rectangle on the image
        				Imgproc.rectangle(source, new Point(minRecX, minRecY), new Point(maxRecX, maxRecY),
        						new Scalar(255, 0, 0), 5);
        				// Give the output stream a new image to display
        				
        				outputStream.putFrame(source);

            }
        }).start();
        
}

	
	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		autoSelected = chooser.getSelected();
		 autoSelected = SmartDashboard.getString("Auto Selector",
		 defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		switch (autoSelected) {
		case customAuto:
			// Put custom auto code here
			break;
		case defaultAuto:
		default:
			// Put default auto code here
			break;
		}
	}
	
	public void disabledInit()
	{
		myGyro.reset();
		myGyro.setSensitivity(voltsPerDegreePerSecond);
		myGyro.calibrate();
	}
	
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		long time=0;
		
		while (isOperatorControl() && isEnabled()) {
			/*if(time<System.currentTimeMillis()||time==0){
				time=System.currentTimeMillis()+5000;
				System.out.println("Gyro:\t " +voltsPerDegreePerSecond+"\t"+ myGyro.getAngle());
				voltsPerDegreePerSecond+=.001;
				myGyro.reset();
				myGyro.setSensitivity(voltsPerDegreePerSecond);
				myGyro.calibrate();
			}*/
			SmartDashboard.putNumber("Gyro Angle", myGyro.getAngle());
			shooterDrive=false;
			double forwardSpeed= stick.getRawAxis(5);
			double turnSpeed = stick.getRawAxis(4);
			drive.arcadeDrive(forwardSpeed, -1*turnSpeed, true);
			SmartDashboard.putNumber("Forward Speed", forwardSpeed);
			SmartDashboard.putNumber("Turn Speed", turnSpeed);
				 // drive with arcade style (use right
										// stick)
			Timer.delay(0.005); // wait for a motor update time
			if(stick.getRawAxis(3) > 0)	//Right trigger
            {
				double speed = SmartDashboard.getNumber("DB/Slider 1", 0.0)/5.0;
				shooter.set(speed);
				SmartDashboard.putNumber("Wheel Speed", speed);
            	shooterDrive=true;
            	
            }
			
			if (stick.getRawButton(1)) // A button
			{
				double speed = SmartDashboard.getNumber("DB/Slider 0", 0.0)/5.0;
				shooter.set(-1*speed);
				SmartDashboard.putNumber("Wheel Speed", speed);
            	shooterDrive=true;
			}
			if (stick.getRawButton(2)) // B button
			{
				double Kp = 0.03;
				double angle = myGyro.getAngle(); // get current heading
		        drive.drive(-.25, -angle*Kp); // drive towards heading 0
		        Timer.delay(0.008);
		        
		        
				System.out.println(myGyro.getAngle());
			}
			if (stick.getRawButton(3)) // X button
			{
				//this is used to create an equation so that the speed of the turn is propprtional to how far away we are
				//this will result in less over correcting
				//RAD
				double MAXTURNSPEED=.25;
				double slope = (2*MAXTURNSPEED)/IMG_WIDTH;
				int goalY=175;
				turnSpeed=.5;
					if(pegY>goalY)
					{
						//make the forward speed equal to the turning speed.
						forwardSpeed=-MAXTURNSPEED;
					}
					else
					{
						forwardSpeed=0;
					}
					turnSpeed = slope*pegX-MAXTURNSPEED;
					SmartDashboard.putNumber("Forward Speed", forwardSpeed);
					SmartDashboard.putNumber("Turn Speed", turnSpeed);
				drive.arcadeDrive(forwardSpeed,turnSpeed);
				
				//THIS WILL FIND THE DISTANCE TO THE TARGET
				/*{
					//d = Tft*FOVpixel/(2*Tpixel*tan(theta)
					//This is the angle of the camera.  THis should be changed;
					double theta=68.5;
					//this is the size IN INCHES of the actual target
					double Tft = 10.25;
					//this is the size in pixels of the target
					double FOVpixel = IMG_HEIGHT;
					double Tpixel=sizeOfTarget;
					double d = Tft*FOVpixel/(2*Tpixel*Math.tan(Math.toRadians(theta)));
					System.out.println("SizeofTarget: " + sizeOfTarget);
					System.out.println("distand: " + d);
					
				}*/
				Timer.delay(0.005);
					
			}
			if (stick.getRawButton(4)) // Y button
			{
				/*for(double i=0;i<1;i+=.001)
				{
				myGyro.reset();
				myGyro.setSensitivity(i);
				myGyro.calibrate();
				//Timer.delay(0.005);
				System.out.println("i: " + i+"\t" + myGyro.getAngle());
				}*/
			}
			
			if ( shooterDrive == false) {
				shooter.set(0);
			}
		}
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
	
	public int getMax(ArrayList<Point> listOfPoints, int z)
	{		
			
		int max=Integer.MIN_VALUE;
		for(int i=0;i<listOfPoints.size();i++)
		{
			int temp=0;
			if(z==0)
			{
				temp=(int) listOfPoints.get(i).x;
			}
			else
			{
				temp=(int) listOfPoints.get(i).y;
			}
			if(temp>max)
				max=temp;
		}
		return max;
	}
	public int getMin(ArrayList<Point> listOfPoints, int z)
	{		
			
		int min=Integer.MAX_VALUE;
		for(int i=0;i<listOfPoints.size();i++)
		{
			int temp=0;
			if(z==0)
			{
				temp=(int) listOfPoints.get(i).x;
			}
			else
			{
				temp=(int) listOfPoints.get(i).y;
			}
			if(temp<min)
				min=temp;
		}
		return min;
	}
	
	public double getRatio(Point[] points)
	{
		double ratio=0;
		ArrayList<Point> temp = (ArrayList<Point>) Arrays.asList(points);
		int maxX = getMax(temp,0);
		int maxY = getMax(temp,1);
		int minX = getMin(temp,0);
		int minY = getMin(temp,1);
		
		
		
		return (maxY-minY)/(maxX-maxX*1.0);
	}
}

