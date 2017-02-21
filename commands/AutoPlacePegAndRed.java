package org.usfirst.frc.team5684.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class AutoPlacePegAndRed extends CommandGroup {

    public AutoPlacePegAndRed() {
    	
    	addSequential(new DriveBackwards(), 1.8);
    	addSequential(new Stop(),.5);
    	addSequential(new WaitCommand(.5));
    	addSequential(new DriveStraight(), .1);
    	addSequential(new Stop(),1);
    	addSequential(new WaitCommand(3));
    	addSequential(new DriveStraight(), 1);
    	addSequential(new Flip(90));
    	addSequential(new DriveStraight(), 1);
    	addSequential(new Stop(),1);
    	addSequential(new ShootWithAgitate(), 1);
    }
}
