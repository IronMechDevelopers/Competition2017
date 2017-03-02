package org.usfirst.frc.team5684.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class AutoPlacePegAndBlue extends CommandGroup {

    public AutoPlacePegAndBlue() {
    	addSequential(new DriveBackwards(), 1.8);
    	addParallel(new ReleaseGear());
    	addSequential(new Stop(),.5);
    	//place the gear
    	addSequential(new WaitCommand(.5));
    	//wait for the gear to be picked up
    	addSequential(new DriveStraight(), .1);
    	addSequential(new Stop(),1);
    	//drive away
    	addSequential(new WaitCommand(3));
    	addSequential(new DriveStraight(), 1);
    	addSequential(new Flip(-90));
    	addSequential(new DriveStaughtTillHit(), 1);
    	addSequential(new Stop(),1);
    	addSequential(new Shoot(Shoot.FORWARD));
    }
}
