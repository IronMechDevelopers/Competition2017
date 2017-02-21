package org.usfirst.frc.team5684.robot.commands;

import org.usfirst.frc.team5684.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveStraight extends Command {
    public DriveStraight() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.drivetrain);
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.drivetrain.tankDrive(0, 0);
		Robot.driveStraightPID.setSetpoint(Robot.imu.getAngleX());
		Robot.driveStraightPID.enable();

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return false || isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("we are done her");
    	Robot.drivetrain.tankDrive(0,0);
    	Robot.driveStraightPID.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
