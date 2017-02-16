package org.usfirst.frc.team5684.robot.triggers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Trigger;

/**
 *
 */
public class LeftSideTriggers extends Trigger {

    public boolean get() {
    	Joystick joystick = new Joystick(0);
        return joystick.getRawAxis(2) > 0 && joystick.getRawAxis(3) > 0;
    }
}
