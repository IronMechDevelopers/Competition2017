package org.usfirst.frc.team5684.robot.subsystems;

import org.usfirst.frc.team5684.robot.RobotMap;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Agitator extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private Victor agitator;
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	agitator = new Victor(RobotMap.agitator);
    }
    
    public void agitate(double speed)
    {
    	agitator.set(speed);
    }
    public void stop()
    {
    	agitator.set(0);
    }
}

