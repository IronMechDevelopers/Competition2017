package org.usfirst.frc.team5684.robot.triggers;

import org.usfirst.frc.team5684.robot.Robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Trigger;

/**
 *
 */
public class RightTrigger extends Trigger {

    public boolean get() {
    	Joystick joystick = new Joystick(0);
        return joystick.getRawAxis(3) > 0;
    }
}
