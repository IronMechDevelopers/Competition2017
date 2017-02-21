package org.usfirst.frc.team5684.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoPlacePegAndStop extends CommandGroup {

	public AutoPlacePegAndStop() {
		addSequential(new DriveBackwards(), 1.8);
	}
}
