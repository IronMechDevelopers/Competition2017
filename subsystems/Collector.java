package org.usfirst.frc.team5684.robot.subsystems;

import org.usfirst.frc.team5684.robot.RobotMap;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Collector extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private Spark collector;
    public void initDefaultCommand() {
        collector=new Spark(RobotMap.collector);
    }
    
    public void setSpeed(double speed)
    {
    	collector.set(speed);
    }
    
    public void stop()
    {
    	collector.set(0);
    }
}

