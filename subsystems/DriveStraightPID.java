package org.usfirst.frc.team5684.robot.subsystems;

import org.usfirst.frc.team5684.robot.Robot;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 *
 */
public class DriveStraightPID extends PIDSubsystem {
    // Initialize your subsystem here
    public DriveStraightPID() {
    	super("DriveStraight",.3,.03,.3,1);
        // Use these to get going:
        // setSetpoint() -  Sets where the PID controller should move the system
        //                  to
        // enable() - Enables the PID controller.
    }
    

    public void initDefaultCommand() {
    	this.setOutputRange(-.75, .75);
    	this.setAbsoluteTolerance(1);
    }

    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
    	System.out.println("Angle: " + Robot.imu.getAngleX());
    	return Robot.imu.getAngleX();
    }

    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);
    	Robot.drivetrain.arcadeDrive(-.66,output);
    }
}
