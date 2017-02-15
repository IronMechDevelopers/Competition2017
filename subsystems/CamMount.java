package org.usfirst.frc.team5684.robot.subsystems;

import org.usfirst.frc.team5684.robot.RobotMap;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class CamMount extends Subsystem {

	Servo cameraBottom;
	Servo cameraTop;

    public void initDefaultCommand() {
        cameraBottom = new Servo(RobotMap.cameraBottom);
        cameraTop = new Servo(RobotMap.cameraTop);
    }
    
    public void setBottomAngle(double angle) {
    	cameraBottom.setAngle(angle);
    	//cameraBottom.set
	}
    
    public void setTopAngle(double angle) {
    	cameraTop.setAngle(angle);
	}
}


