package org.usfirst.frc.team5684.robot.subsystems;

import org.usfirst.frc.team5684.robot.RobotMap;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Arm extends Subsystem{
	private Spark arm;
	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		arm = new Spark(RobotMap.arm);
	}
	
	public void raiseArm()
	{
		arm.set(.25);
	}
	
	public void lowerArm()
	{
		arm.set(-.25);
	}
	public void stop()
	{
		arm.set(0);
	}

}
