package org.usfirst.frc.team5684.robot.commands;

import org.usfirst.frc.team5684.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class StartShooter extends Command {
	private long startTime;
	private long runTime;
	
    public StartShooter() {
        requires(Robot.shooter);
        runTime=5000;
    }

    // Called just before this Command runs the first time
    public void initialize() {
    	System.out.println("Initialize from StartShooter");
    	startTime=System.currentTimeMillis();
    }

    // Called repeatedly when this Command is scheduled to run
    public void execute() {
    	System.out.println("execute from StartShooter");
    	Robot.shooter.setSpeed(.50);
    }

    // Make this return true when this Command no longer needs to run execute()
    public boolean isFinished() {
    	if(System.currentTimeMillis()>startTime+runTime)
        {
        	System.out.println("we are done");
        	return true;
        }
        else
        	return false;
    }

    // Called once after isFinished returns true
    public void end() {
    	Robot.shooter.setSpeed(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.shooter.setSpeed(0);
    }
}
