package org.usfirst.frc.team5684.robot.triggers;

import org.usfirst.frc.team5684.robot.RobotMap;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;

/**
 *
 */
public class RightTrigger extends Button {
	Joystick joystick;
	public RightTrigger(Joystick joystick)
	{
		this.joystick=joystick;
	}
    public boolean get() {
		return joystick.getRawAxis(RobotMap.rightTrigger)>0 && joystick.getRawAxis(RobotMap.leftTrigger)==0;
    }
}
