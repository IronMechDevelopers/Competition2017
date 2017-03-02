package org.usfirst.frc.team5684.robot;

import org.usfirst.frc.team5684.robot.commands.Agitate;
import org.usfirst.frc.team5684.robot.commands.AutoForward;
import org.usfirst.frc.team5684.robot.commands.AutoPlacePegAndRed;
import org.usfirst.frc.team5684.robot.commands.AutoPlacePegAndStop;
import org.usfirst.frc.team5684.robot.commands.Camera;
import org.usfirst.frc.team5684.robot.commands.Climb;
import org.usfirst.frc.team5684.robot.commands.ClimbFaster;
import org.usfirst.frc.team5684.robot.commands.Collect;
import org.usfirst.frc.team5684.robot.commands.DecreaseShooterSpeed;
import org.usfirst.frc.team5684.robot.commands.DriveStraight;
import org.usfirst.frc.team5684.robot.commands.Everything;
import org.usfirst.frc.team5684.robot.commands.Flip;
import org.usfirst.frc.team5684.robot.commands.FollowPeg;
import org.usfirst.frc.team5684.robot.commands.IncreaseShooterSpeed;
import org.usfirst.frc.team5684.robot.commands.LowerArm;
import org.usfirst.frc.team5684.robot.commands.RaiseArm;
import org.usfirst.frc.team5684.robot.commands.Shoot;
import org.usfirst.frc.team5684.robot.commands.ShootWithAgitate;
import org.usfirst.frc.team5684.robot.commands.Square;
import org.usfirst.frc.team5684.robot.commands.ZeroCamera;
import org.usfirst.frc.team5684.robot.triggers.BackwardButton;
import org.usfirst.frc.team5684.robot.triggers.ForwardButton;
import org.usfirst.frc.team5684.robot.triggers.RightTrigger;
import org.usfirst.frc.team5684.robot.triggers.BothTriggers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Trigger;
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
	 Joystick stick = new Joystick(1);
	 Button xButton = new ForwardButton(joystick,RobotMap.leftTrigger,RobotMap.xButton);
	 Button xBackButton = new BackwardButton(joystick,RobotMap.leftTrigger,RobotMap.xButton);
	 
	 Button yButton = new ForwardButton(joystick,RobotMap.leftTrigger,RobotMap.yButton);
	 Button yBackButton = new BackwardButton(joystick,RobotMap.leftTrigger,RobotMap.yButton);
	 
	 Button bButton = new ForwardButton(joystick,RobotMap.leftTrigger,RobotMap.bButton);
	 Button bBackButton = new BackwardButton(joystick,RobotMap.leftTrigger,RobotMap.bButton);
	 
	 Button aButton = new ForwardButton(joystick,RobotMap.leftTrigger,RobotMap.aButton);
	 Button aBackButton = new BackwardButton(joystick,RobotMap.leftTrigger,RobotMap.aButton);
	 
	 Button rightBumper= new ForwardButton(joystick,RobotMap.leftTrigger,RobotMap.rightBumper);
	 Button rightBackBumper= new BackwardButton(joystick,RobotMap.leftTrigger,RobotMap.rightBumper);
	 
	 Button leftBumper= new ForwardButton(joystick,RobotMap.leftTrigger,RobotMap.leftBumper);
	 Button leftBackBumper= new BackwardButton(joystick,RobotMap.leftTrigger,RobotMap.leftBumper);
	 
	 Button RightTrigger = new RightTrigger(joystick);
	 Button BothTriggers = new BothTriggers(joystick);
	 
	 Button fire = new JoystickButton(stick,1);
	 Button thumb = new JoystickButton(stick,2);
	 
	 Button JSShoot = new JoystickButton(stick,1);
	 Button button7 = new JoystickButton(stick,7);
	 
	 Button button3 = new JoystickButton(stick,3);
	 Button button4 = new JoystickButton(stick,4);
	 Button button5 = new JoystickButton(stick,5);
	 Button button6 = new JoystickButton(stick,6);
	 
	 Button button8 = new JoystickButton(stick,8);
	 Button button9 = new JoystickButton(stick,9);
	 Button button10 = new JoystickButton(stick,10);
	 Button button2 = new JoystickButton(stick,2);
	 
	 
	
	 
	 //To control speed of collector
	 //Button backButton = new JoystickButton(joystick,10);	//Not sure value 10 is correct
	 

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
			
			
			xButton.whileHeld(new ShootWithAgitate());
			xBackButton.whileHeld(new Shoot(Shoot.BACKWARD));
			
			aBackButton.whenPressed(new Square());
			
			
			leftBumper.whileHeld(new Agitate(Agitate.FORWARD));
			leftBackBumper.whileHeld(new Agitate(Agitate.BACKWARD));
			
			rightBumper.whileHeld(new Collect(Collect.FORWARD));
			rightBackBumper.whileHeld(new Collect(Collect.BACKWARD));
			
			yButton.whenPressed(new IncreaseShooterSpeed());
			aButton.whenPressed(new DecreaseShooterSpeed());
			
			RightTrigger.whileHeld(new Climb(Climb.SLOWSPEEDCLIMB));
			bButton.whileHeld(new Climb(Climb.SLOWSPEEDLOWER));
			BothTriggers.whileHeld(new ClimbFaster());
			
			JSShoot.toggleWhenPressed(new ShootWithAgitate());
			button7.whenPressed(new IncreaseShooterSpeed());
			button8.whenPressed(new DecreaseShooterSpeed());
			
			button9.toggleWhenPressed(new Collect(Collect.FORWARD));
			button10.toggleWhenPressed(new Collect(Collect.BACKWARD));
			
			//button9.whileHeld(new Collect(Collect.FORWARD,stick.getRawAxis(1)/1.0));
			
			button2.whileHeld(new Climb(Climb.SLOWSPEEDCLIMB));
			
			button3.whileHeld(new Climb(Climb.SLOWSPEEDCLIMB));
			button5.whileHeld(new ClimbFaster());
			
			button6.whileHeld(new Climb(Climb.SLOWSPEEDLOWER));
			
			
		}
		
		

		public Joystick getJoystick() {
			return joystick;
		}
		
		
}
