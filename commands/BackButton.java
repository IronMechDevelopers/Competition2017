package org.usfirst.frc.team5684.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

public class BackButton extends Command {

	int speed;

	public BackButton()
	{
		speed = 0;
	}

	protected void execute()
	{
		speed--;
	}

	/*
	 * Return speed decreased by 1
	 */
	public int getSpeed()
	{
		return speed;
	}
	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}
}