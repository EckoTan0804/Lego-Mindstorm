package mission;

import ev3Robot.Robot;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.robotics.Color;
import util.BrickScreen;

public class Labyrinth {
	
	private final int RED = 0;
	private final int GREEN = 1;
	private final int BLUE = 2;
	private final int WHITE = 3;
	private final int BLACK = 4;
	
	
	private Robot robot;
	private boolean beginLabyrinth = false;
	private boolean endLabyrinth = false;
	
	

	public Labyrinth(Robot robot) {
		super();
		this.robot = robot;
	}

	public void startLabyrinth() {

		BrickScreen.clearScreen();

		this.robot.changeSettingsForLabyrinth();

		try {
			Thread.sleep(20);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		float speed = 150f; // TODO: need to adjust the speed
		this.robot.getDrive().setMotorSpeed(speed, speed);
		this.robot.getDrive().goForwardWithMotors();
		float leftMotorSpeed = 0f;
		float rightMotorSpeed = 0f;

		
		while (Button.LEFT.isUp() && (!endLabyrinth)) {
			
			BrickScreen.clearScreen();
			
			float[] rgb = this.robot.getSensors().getColorArray();
			float red = rgb[RED] * 255;
			float green = rgb[GREEN] * 255;
			float blue = rgb[BLUE] * 255;
			
			BrickScreen.show("r: " + red);
			BrickScreen.show("g: " + green);
			BrickScreen.show("b: " + blue);
		
			if (red > 15) {
				if (red > 23 && green < 35 && blue < 23) {	 // red
					this.robot.getDrive().travel(3);
					this.robot.getDrive().turnRight(97);
					BrickScreen.show("Red");	
				} else {	 //white
					leftMotorSpeed = 1.6f * speed;
					rightMotorSpeed = 1.0f * speed;
					BrickScreen.show("White");
					this.adjustRobotMovement(robot, leftMotorSpeed, rightMotorSpeed);
				}
			} else {			//blue
				if (blue > 16 && green <= 21) {
					leftMotorSpeed = 0f;
					rightMotorSpeed = 0f;
					BrickScreen.show("Blue");
					this.adjustRobotMovement(robot, leftMotorSpeed, rightMotorSpeed);
				} else {		//black
					leftMotorSpeed = (float)(-0.5) * speed;
					rightMotorSpeed = (float)1.1f * speed;
					this.adjustRobotMovement(robot, leftMotorSpeed, rightMotorSpeed);
					BrickScreen.show("Black");
				}
			}
			
		}
	}
	
	private void adjustRobotMovement(Robot robot, float leftTargetSpeed, float rightTargetSpeed) {

		/* print the target speed of left and right motors on the brick's screen */
		BrickScreen.show("L= " + leftTargetSpeed);
		BrickScreen.show("R= " + rightTargetSpeed);

		robot.getLeftMotor().startSynchronization();
		robot.getRightMotor().startSynchronization();

		if (leftTargetSpeed < 0) {
			robot.getDrive().setLeftMotorSpeed(-leftTargetSpeed);
			robot.getLeftMotor().backward();
		} else {
			robot.getDrive().setLeftMotorSpeed(leftTargetSpeed);
			robot.getLeftMotor().forward();
		}
		if (rightTargetSpeed < 0) {
			robot.getDrive().setRightMotorSpeed(-rightTargetSpeed);
			robot.getRightMotor().backward();
		} else {
			robot.getDrive().setRightMotorSpeed(rightTargetSpeed);
			robot.getRightMotor().forward();
		}

		robot.getLeftMotor().endSynchronization();
		robot.getRightMotor().endSynchronization();
	}

	private int getColor(float redVal, float greenVal, float blueVal) {
		return 0;
	}



}
