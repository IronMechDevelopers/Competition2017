package org.usfirst.frc.team5684.robot.commands;

import org.usfirst.frc.team5684.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class IncreaseShooterSpeed extends Command {
	double temp;
	boolean ranOnce;
    public IncreaseShooterSpeed() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	ranOnce=false;
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	temp = SmartDashboard.getNumber(RobotMap.shooterSlider, 2.5);
    	ranOnce=true;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return ranOnce;
    }

    // Called once after isFinished returns true
    protected void end() {
    	SmartDashboard.putNumber(RobotMap.shooterSlider, temp+.05);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
