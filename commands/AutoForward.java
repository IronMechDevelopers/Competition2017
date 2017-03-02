package org.usfirst.frc.team5684.robot.commands;

import org.usfirst.frc.team5684.robot.Robot;
import org.usfirst.frc.team5684.robot.subsystems.Turn;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoForward extends CommandGroup {

    public AutoForward() {
    	addSequential(new DriveStraight(),2.75);
    	addParallel(new ReleaseGear());
    	addSequential(new Stop(),1);
    }

}
