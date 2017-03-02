package org.usfirst.frc.team5684.robot.commands;

import org.usfirst.frc.team5684.robot.Robot;
import org.usfirst.frc.team5684.robot.RobotMap;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Shoot extends Command {
	public static int FORWARD = 1;
	public static int BACKWARD = -1;
	private int direction;
	
    public Shoot(int direction) {
        requires(Robot.shooter);
        this.direction=direction;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    
    public void execute() {
    	double speed = direction*SmartDashboard.getNumber(RobotMap.shooterSlider, 2.5);
		Robot.shooter.setSpeed(-1*speed/5.0);
		SmartDashboard.putNumber(RobotMap.shooterSlider, speed);

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
    	Robot.shooter.stop();
    }
}
