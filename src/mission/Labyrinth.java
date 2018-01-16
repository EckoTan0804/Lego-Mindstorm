package mission;

import ev3Robot.Robot;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.robotics.Color;
import util.BrickScreen;

public class Labyrinth {

	private final int RED = 0;
	private final int GREEN = 1;
	private final int BLUE = 2;

	private Robot robot;
	private boolean beginLabyrinth = false;
	private boolean endLabyrinth = false;

	public Labyrinth(Robot robot) {
		super();
		this.robot = robot;
	}

	public void startLabyrinth() {
		
		Sound.beep();

		BrickScreen.clearScreen();

		this.robot.changeSettingsForLabyrinth();

		try {
			Thread.sleep(20);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		this.enterLabyrinth();

		float speed = 150f; // TODO: need to adjust the speed
		this.robot.getDrive().setMotorSpeed(speed, speed);
		this.robot.getDrive().goForwardWithMotors();

		float leftTargetSpeed = 0f;
		float rightTargetSpeed = 0f;

		while (Button.LEFT.isUp() && (!endLabyrinth)) {

			BrickScreen.clearScreen();

			float[] rgb = this.getColorArrayInRgbMode();
			float red = rgb[RED];
			float green = rgb[GREEN];
			float blue = rgb[BLUE];

			BrickScreen.show("r: " + red);
			BrickScreen.show("g: " + green);
			BrickScreen.show("b: " + blue);

			if (red > 15) {
				if (red > 23 && green < 35 && blue < 23) { // red
					this.robot.getDrive().travel(3);
					this.robot.getDrive().turnRight(97);
					BrickScreen.show("Red");

				} else { // white
					leftTargetSpeed = 1.6f * speed;
					rightTargetSpeed = 1.0f * speed;
					BrickScreen.show("White");
					this.robot.getDrive().adjustRobotMovement(leftTargetSpeed, rightTargetSpeed);
				}
			} else { // blue
				if (blue > 16 && green <= 23) {
					if (!this.beginLabyrinth) {
						Sound.beep();
						this.beginLabyrinth = true;
						this.robot.getDrive().travel(15);
						this.robot.getDrive().turnRight(90);
					} else if (blue > 18) { // TODO: the parameter should be adjusted and make sure that it won't go
											// wrong
						leftTargetSpeed = 0f;
						rightTargetSpeed = 0f;
						BrickScreen.show("Blue");
						this.robot.getDrive().adjustRobotMovement(leftTargetSpeed, rightTargetSpeed);
						this.endLabyrinth = true;
						Sound.beepSequenceUp();
					}
				} else { // black
					leftTargetSpeed = (float) (-0.5) * speed;
					rightTargetSpeed = (float) 1.1f * speed;
					this.robot.getDrive().adjustRobotMovement(leftTargetSpeed, rightTargetSpeed);
					BrickScreen.show("Black");
				}
			}

		}

		Sound.beepSequenceUp();
	}

	private void enterLabyrinth() {

		float[] rgb = this.getColorArrayInRgbMode();
		float red = rgb[RED];
		float green = rgb[GREEN];
		float blue = rgb[BLUE];

		BrickScreen.show("r: " + red);
		BrickScreen.show("g: " + green);
		BrickScreen.show("b: " + blue);

		if (red > 15 && blue > 16 && green <= 21) { /* blue */
			Sound.beep();
			this.beginLabyrinth = true;
			this.robot.getDrive().travel(15);
			this.robot.getDrive().turnRight(90);
		}
	}

	private float[] getColorArrayInRgbMode() {
		float[] rgb = this.robot.getSensors().getColorArray();
		for (int i = 0; i < rgb.length; i++) {
			rgb[i] *= 255;
		}
		return rgb;
	}

	public void reset() {
		this.beginLabyrinth = false;
		this.endLabyrinth = false;
	}
	
	public void fromLabyrinthToBridge() {
		
		// TODO: add touch sensor determination
		this.robot.getDrive().travel(2);
	}

	public Robot getRobot() {
		return robot;
	}

	public void setRobot(Robot robot) {
		this.robot = robot;
	}

	public boolean isBeginLabyrinth() {
		return beginLabyrinth;
	}

	public void setBeginLabyrinth(boolean beginLabyrinth) {
		this.beginLabyrinth = beginLabyrinth;
	}

	public boolean isEndLabyrinth() {
		return endLabyrinth;
	}

	public void setEndLabyrinth(boolean endLabyrinth) {
		this.endLabyrinth = endLabyrinth;
	}

}
