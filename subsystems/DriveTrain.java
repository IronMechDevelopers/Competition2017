package org.usfirst.frc.team5684.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team5684.robot.Robot;
import org.usfirst.frc.team5684.robot.RobotMap;
import org.usfirst.frc.team5684.robot.commands.DriveWithJoystick;
import edu.wpi.first.wpilibj.command.Command;

/**
 * The DriveTrain subsystem controls the robot's chassis and reads in
 * information about it's speed and position.
 */
public class DriveTrain extends Subsystem {
	// Subsystem devices
	private static RobotDrive drive;
	public static int forward = 1;
	// private AnalogGyro gyro = Robot.myGyro;

	public DriveTrain() {
		// Configure drive motors
		// Configure the RobotDrive to reflect the fact that all our motors are
		// wired backwards and our drivers sensitivity preferences.
		drive = new RobotDrive(RobotMap.leftWheels, RobotMap.rightWheels);
		//drive.setSafetyEnabled(true);
		//drive.setExpiration(0.1);
		//drive.setSensitivity(0.5);
		//drive.setMaxOutput(1.0);

		// Configure gyro
		if (Robot.isReal()) {
			// gyro.setSensitivity(0.007); // TODO: Handle more gracefully?
		}
		// LiveWindow.addSensor("DriveTrain", "Gyro", gyro);
	}

	/**
	 * When other commands aren't using the drivetrain, allow tank drive with
	 * the joystick.
	 */
	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new DriveWithJoystick());
		//drive.setSafetyEnabled(false);
	}

	public void arcadeDrive(Joystick joy) {
		 drive.arcadeDrive(forward*joy.getRawAxis(5),-1*forward*joy.getRawAxis(4),true);
	}
	
	public void arcadeDrive(double one, double two) {
		 drive.arcadeDrive(one, two);
	}


	/**
	 * @param leftAxis
	 *            Left sides value
	 * @param rightAxis
	 *            Right sides value
	 */
	public void tankDrive(double leftAxis, double rightAxis) {
		drive.tankDrive(leftAxis, rightAxis);
	}

	public void RadTestDrive(Joystick joy) {
		if (Math.abs(joy.getZ()) < .2)
			drive.arcadeDrive(joy.getY(), joy.getX(), true);
		else
			drive.arcadeDrive(0, -1*joy.getZ());
	}

	/**
	 * Stop the drivetrain from moving.
	 */
	public void stop() {
		drive.tankDrive(0, 0);
	}

	/**
	 * @return The current angle of the drivetrain.
	 */
	// public double getAngle() {
	// return gyro.getAngle();
	// }

	public static void turn(double d) {
		drive.arcadeDrive(0, d);
	}
}