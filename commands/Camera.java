package org.usfirst.frc.team5684.robot.commands;

import org.usfirst.frc.team5684.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Camera extends Command {

    public Camera() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.camMount);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double x = Robot.oi.getJoystick().getRawAxis(0);
    	double y = Robot.oi.getJoystick().getRawAxis(1);
    	Robot.camMount.setBottomAngle(90*x+90);
    	Robot.camMount.setTopAngle(90*y+90);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.camMount.setBottomAngle(0);
    	Robot.camMount.setTopAngle(0);
    }
}

