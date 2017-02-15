package org.usfirst.frc.team5684.robot.subsystems;

import org.usfirst.frc.team5684.robot.Robot;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 *
 */
public class CenterPeg extends PIDSubsystem {

    // Initialize your subsystem here
    public CenterPeg() {
    	super("Center Peg",1,0,0);
    	this.setOutputRange(-.75, .75);
    	//this.setAbsoluteTolerance(5);
        // Use these to get going:
        // setSetpoint() -  Sets where the PID controller should move the system
        //                  to
        // enable() - Enables the PID controller.
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
    	System.out.println(Robot.centerX-Robot.IMG_WIDTH);
        return Robot.centerX-Robot.IMG_WIDTH;
    }

    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);
    	System.out.println("\toutput:  " + output);
    	DriveTrain.turn(output);
    }
}
