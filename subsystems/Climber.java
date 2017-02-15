package org.usfirst.frc.team5684.robot.subsystems;

import org.usfirst.frc.team5684.robot.RobotMap;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Climber extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private Spark climber;

    public void initDefaultCommand() {
        climber = new Spark(RobotMap.climber);
    }

	public void setSpeed(double speed) {
		climber.set(speed);
	}
	public void stop() {
		climber.set(0);
	}
}

