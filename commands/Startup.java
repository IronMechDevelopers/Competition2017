package org.usfirst.frc.team5684.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class Startup extends CommandGroup {

    public Startup() {
    	addSequential(new Shoot(),4);
    	addParallel(new DriveStraight(),2);
    	addSequential(new Collect(Collect.FORWARD),4);
    	addSequential(new Collect(Collect.BACKWARD),4);
    	addSequential(new Climb(Climb.SLOWSPEEDCLIMB),4);
    	addSequential(new Climb(Climb.FASTSPEEDCLIMB),4);
    }
}
