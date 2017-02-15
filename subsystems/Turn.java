package org.usfirst.frc.team5684.robot.subsystems;

import org.usfirst.frc.team5684.robot.Robot;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 *
 */
public class Turn extends PIDSubsystem {

    // Initialize your subsystem here
    public Turn() {
        // Use these to get going:
        // setSetpoint() -  Sets where the PID controller should move the system
        //                  to
        // enable() - Enables the PID controller.
    	
    	super("Turn",.3,10,0);
    		
    }

    public void initDefaultCommand() {
    	Robot.myGyro.reset();
    	Robot.myGyro.setSensitivity(Robot.voltsPerDegreePerSecond);
    	Robot.myGyro.calibrate();
    	this.setOutputRange(-.75, .75);
    	this.setAbsoluteTolerance(5);
 
    }

    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
        return Robot.myGyro.getAngle();
    }

    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);
    	System.out.println("At: " + Robot.myGyro.getAngle() +"\t setPoint: " + this.getSetpoint());
    	DriveTrain.turn(output);
    }
}
