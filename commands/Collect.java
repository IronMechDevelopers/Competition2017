package org.usfirst.frc.team5684.robot.commands;

import org.usfirst.frc.team5684.robot.OI;
import org.usfirst.frc.team5684.robot.Robot;
import org.usfirst.frc.team5684.robot.RobotMap;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Collect extends Command {
	
	public static int FORWARD = 1;
	public static int BACKWARD = -1;
	int direction;
	
    public Collect(int direction) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.collector);
    	this.direction=direction;
    }

    public Collect(int direction, double d) {
		Robot.collector.setSpeed(-1*d*direction);
		SmartDashboard.putNumber("collector Speed", d);
	}

	// Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double speed = new Joystick(1).getRawAxis(1)*5.0;
		Robot.collector.setSpeed(speed/5.0*direction);
		SmartDashboard.putNumber("collector Speed", Math.abs(speed/5.0*direction));
    	
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
    	Robot.collector.stop();
    }
}
