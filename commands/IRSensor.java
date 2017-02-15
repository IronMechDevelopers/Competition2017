package org.usfirst.frc.team5684.robot.commands;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class IRSensor extends Command {
	private Ultrasonic ultra;
    public IRSensor() {
    	ultra= new Ultrasonic(0,1);
		ultra.setAutomaticMode(true);
		
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double range = ultra.getRangeInches();
		System.out.print("Hello\t");
		System.out.print(range);
		System.out.println("\tGoodBye");
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
    	System.out.println("Hello we are done here");
    }
}
