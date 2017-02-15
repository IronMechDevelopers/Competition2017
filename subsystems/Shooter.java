package org.usfirst.frc.team5684.robot.subsystems;

import org.usfirst.frc.team5684.robot.RobotMap;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Shooter extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private Spark shooter;

    public void initDefaultCommand() {
    	shooter = new Spark(RobotMap.shooter);
    }
    
    public void setSpeed(double speed) {
    	shooter.set(speed);
	}
	public void stop() {
		shooter.set(0);
	}
}

