package org.usfirst.frc.team5684.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The Shooter subsystem handles shooting. The mechanism for shooting is
 * slightly complicated because it has to pneumatic cylinders for shooting, and
 * a third latch to allow the pressure to partially build up and reduce the
 * effect of the airflow. For shorter shots, when full power isn't needed, only
 * one cylinder fires.
 *
 * NOTE: Simulation currently approximates this as as single pneumatic cylinder
 * and ignores the latch.
 */
public class Shooter extends Subsystem {
	// Devices
	VictorSP shooter=new VictorSP(4);

	public Shooter() {
		// 
	}
	/**
	 * No default command.
	 */
	@Override
	public void initDefaultCommand() {
	}
	public void Shoot()
	{
		
	}
	}