package org.usfirst.frc.team5684.robot.triggers;

import org.usfirst.frc.team5684.robot.RobotMap;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;

/**
 *
 */
public class BackwardButton extends Button {
	int trigger;
	int button;
	Joystick joystick;
	
	public BackwardButton(Joystick joystick, int trigger, int button)
	{
		this.joystick=joystick;
		this.trigger=trigger;
		this.button=button;
	}
	
    public boolean get() {
    	return joystick.getRawAxis(trigger) > 0 && joystick.getRawButton(button);
    }
}
