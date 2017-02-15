package org.usfirst.frc.team5684.robot.commands;

import org.usfirst.frc.team5684.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TestShooter extends Command {
	private long startTime;
	private long runTime;
    public TestShooter() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.shooter);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	startTime=System.currentTimeMillis();
    	runTime=500;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.shooter.setSpeed(.25);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        if(System.currentTimeMillis()>=startTime+runTime)
        	return true;
        else
        	return false;
        
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.shooter.setSpeed(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.shooter.setSpeed(0);
    }
}
