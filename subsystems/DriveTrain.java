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
	//private AnalogGyro gyro = Robot.myGyro;

	public DriveTrain() {
		// Configure drive motors
		// Configure the RobotDrive to reflect the fact that all our motors are
		// wired backwards and our drivers sensitivity preferences.
		drive = new RobotDrive(RobotMap.leftWheels,RobotMap.rightWheels);
		drive.setSafetyEnabled(true);
		drive.setExpiration(0.1);
		drive.setSensitivity(0.5);
		drive.setMaxOutput(1.0);

		// Configure gyro
		if (Robot.isReal()) {
			//gyro.setSensitivity(0.007); // TODO: Handle more gracefully?
		}
		//LiveWindow.addSensor("DriveTrain", "Gyro", gyro);
	}

	/**
	 * When other commands aren't using the drivetrain, allow tank drive with
	 * the joystick.
	 */
	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new DriveWithJoystick());
		drive.setSafetyEnabled(false);
	}
	
	public void arcadeDrive(Joystick joy) {
		drive.arcadeDrive(joy.getRawAxis(5),-1*joy.getRawAxis(4),true);
	}

	/**
	 * @param joy
	 *            PS3 style joystick to use as the input for tank drive.
	 */
	public void tankDrive(Joystick joy) {
		drive.tankDrive(joy.getY(), joy.getX(),true);
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

	/**
	 * Stop the drivetrain from moving.
	 */
	public void stop() {
		drive.tankDrive(0, 0);
	}

	/**
	 * @return The current angle of the drivetrain.
	 */
	//public double getAngle() {
		//return gyro.getAngle();
	//}

	public static void turn(double d) {
		drive.arcadeDrive(0, d); 	
	}
}