package org.usfirst.frc.team5684.robot.subsystems;

import org.usfirst.frc.team5684.robot.Robot;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    	
    	super("Turn",.3,0,0);
    		
    }

    public void initDefaultCommand() {
    	this.setOutputRange(-.66, .66);
    	this.setAbsoluteTolerance(1);
    	
 
    }

    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
        return Robot.imu.getAngleX();
    }

    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);
    	DriveTrain.turn(output);
    }
}
