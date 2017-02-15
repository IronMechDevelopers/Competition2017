package org.usfirst.frc.team5684.robot.commands;

import org.usfirst.frc.team5684.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Flip extends Command {
	private double setpoint;
    public Flip(double setpoint) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.setpoint=setpoint;
    	requires(Robot.drivetrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("Hello I'm Mr. MeSeeks Look at me!!");
    	Robot.myGyro.reset();
    	this.setpoint += Robot.myGyro.getAngle();
    	System.out.println("At: " + Robot.myGyro.getAngle() +"\t setPoint: " + setpoint);
		Robot.turn.setSetpoint(setpoint);
		Robot.turn.enable();
		
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	//boolean temp = Math.abs(setpoint-Robot.myGyro.getAngle())<2;
    	//if(temp)
    		//Robot.turn.disable();
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
