package org.usfirst.frc.team5684.robot.commands;

import org.usfirst.frc.team5684.robot.Robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveStraight extends Command {
	private long startTime;
	private long runTime;
	private AnalogGyro gyro;
	double Kp=.3;
    public DriveStraight() {
        requires(Robot.drivetrain);
        runTime=5000;
        gyro=Robot.myGyro;
        gyro.reset();
        gyro.calibrate();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	startTime=System.currentTimeMillis();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	System.out.println("HELLO");
    	double angle = gyro.getAngle();
    	System.out.println("Inside");
    	Robot.drivetrain.tankDrive(-.25,-angle*Kp);
    	System.out.println("done");
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	
        if(System.currentTimeMillis()>startTime+runTime)
        {
        	System.out.println("we are done");
        	return true;
        }
        else
        	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drivetrain.tankDrive(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.drivetrain.tankDrive(0, 0);
    }
}
