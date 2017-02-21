package org.usfirst.frc.team5684.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ClimbFaster extends CommandGroup {

    public ClimbFaster() {

    addSequential(new Climb(Climb.SLOWSPEEDCLIMB),.5);
    addSequential(new Climb(Climb.SLOWSPEEDCLIMB+1*.05),.5);
    addSequential(new Climb(Climb.SLOWSPEEDCLIMB+2*.05),.5);
    addSequential(new Climb(Climb.SLOWSPEEDCLIMB+3*.05),.5);
    addSequential(new Climb(Climb.SLOWSPEEDCLIMB+4*.05),.5);
    addSequential(new Climb(Climb.SLOWSPEEDCLIMB+5*.05));
        
    }
}
