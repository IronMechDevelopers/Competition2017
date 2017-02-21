package org.usfirst.frc.team5684.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class Startup extends CommandGroup {

    public Startup() {
    	addSequential(new Shoot(Shoot.FORWARD),2);
    	addSequential(new Shoot(Shoot.BACKWARD),2);
    	//addParallel(new DriveStraight(),2);
    	addSequential(new Collect(Collect.FORWARD),2);
    	addSequential(new Collect(Collect.BACKWARD),2);
    	addSequential(new Climb(Climb.SLOWSPEEDCLIMB),2);
    	addSequential(new Climb(Climb.FASTSPEEDCLIMB),2);
    	
    }
}
