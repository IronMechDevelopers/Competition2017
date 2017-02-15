package org.usfirst.frc.team5684.robot;

import org.usfirst.frc.team5684.robot.commands.Camera;
import org.usfirst.frc.team5684.robot.commands.Climb;
import org.usfirst.frc.team5684.robot.commands.Collect;
import org.usfirst.frc.team5684.robot.commands.DriveStraight;
import org.usfirst.frc.team5684.robot.commands.Flip;
import org.usfirst.frc.team5684.robot.commands.FollowPeg;
import org.usfirst.frc.team5684.robot.commands.IRSensor;
import org.usfirst.frc.team5684.robot.commands.LowerArm;
import org.usfirst.frc.team5684.robot.commands.RaiseArm;
import org.usfirst.frc.team5684.robot.commands.Shoot;
import org.usfirst.frc.team5684.robot.commands.StartShooter;
import org.usfirst.frc.team5684.robot.commands.ZeroCamera;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	 Joystick joystick = new Joystick(0);
	 Button aButton = new JoystickButton(joystick,1);
	 Button xButton = new JoystickButton(joystick,3);
	 Button yButton = new JoystickButton(joystick,4);
	 Button bButton = new JoystickButton(joystick,2);
	 Button rightShoulderButton = new JoystickButton(joystick,6);
	 Button leftShoulderButton = new JoystickButton(joystick,5);
	 Button rightStickIn = new JoystickButton(joystick,9);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
	 
		public OI() {
			aButton.whileHeld(new RaiseArm());
			bButton.whileHeld(new ZeroCamera());
			xButton.whenPressed(new StartShooter());
			yButton.whenPressed(new Camera());
			rightShoulderButton.whileHeld(new Climb());
			leftShoulderButton.whileHeld(new Collect());
			rightStickIn.whenPressed(new StartShooter());
			
			/*
			 * xButton on/off for shooter
			 * right Trigger ground collector suck in balls
			 * left Trigger ground collector spit balls out
			 *  Right Bumper slow climb
			 *  left bunmper medium climb
			 *  both bumpers fast climb
			 */
			
			/*
			 * xButton on/off for shooter
			 * a Button ground collector suck in balls
			 * b Button ground collector spit balls out
			 *  Right Bumper slow climb
			 *  left bunmper medium climb
			 *  both bumpers fast climb
			 */
			
			/*
			 * left joystick drives
			 * left trigger collector with y button tuggles direction
			 * right trigger climber with b button tuggles speed contorl by either B or right joystick
			 * shooting is x while pressed
			 *  Left trigger change what is forwards
			 * 
			 */
			
		}
		
		

		public Joystick getJoystick() {
			return joystick;
		}
		
		
}
