package mission;

import ev3Robot.Robot;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.robotics.Color;
import util.BrickScreen;

public class Labyrinth {

	private Robot robot;
	private boolean endLabyrinth = false;

	public Labyrinth(Robot robot) {
		super();
		this.robot = robot;
	}

	public void startLabyrinth() {

		this.robot.changeSettingsForLabyrinth();
		BrickScreen.show("Labyrinth");

		float speed = this.robot.getLeftMotor().getMaxSpeed();
		this.robot.getDrive().setMotorSpeed(speed, speed);
		this.robot.getDrive().goForwardWithMotors();

		while (Button.LEFT.isUp() && (!endLabyrinth)) {

			// int colorVal = (int) this.robot.getSensors().getColor();
			int colorID = this.robot.getColorID();
			BrickScreen.show("color = " + colorID);
//			LCD.drawString("color = " + colorID, 0, 1);

			switch (colorID) {

			case Color.RED:
				/*
				 * the robot reaches a cross or a corner, needs to do a 90 degree rotation
				 */
				this.robot.getDrive().stopWithMotors();
				this.robot.getDrive().turnRight(90);

				if (this.robot.getColorID() != Color.WHITE) {

					if (this.robot.getColorID() == Color.BLACK) {
						/*
						 * If the robot sees BLACK after it has turned right for 90 degree, that means,
						 * it has reached an dead end.
						 */
						this.robot.getDrive().turnRight(90);
					} else {
						/*
						 * If the robot can not find the white line or BLACK after it has turned right
						 * for 90 degree, that means, the white line is on its left side so it should
						 * turn left for 180 degree to look for the line
						 */
						this.robot.getDrive().turnLeft(180);
					}

				}
				break;

			case Color.BLACK: // TODO: need to measure the color ID of the ground
				
				/*
				 * The robot has drifted away from the white line
				 */

				int arc = 0;
				boolean found = false;
				while (arc < 90 && !found) {
					this.robot.getDrive().turnRight(10);
					found = this.robot.getColorID() == Color.WHITE;
					arc += 10;
				}

				if (!found) {
					this.robot.getDrive().turnLeft(arc + 10);
					findLine();
				}
				break;

			case Color.BLUE:
				/*
				 * the robot reaches the destination, finish!
				 */
				this.endLabyrinth = true;
				break;

			case Color.WHITE:
				/*
				 * the robot runs on the white line, just go straight
				 */
				this.robot.getDrive().goForwardWithPilot();
				break;

			default:
				break;

			}

		}
	}
	
	private void findLine() {
		boolean found = false;
		while (!found) {
			this.robot.getDrive().travel(7);
			int arc = 0;
			while (arc < 90 && !found) {
				this.robot.getDrive().turnRight(10);
				found = this.robot.getColorID() == Color.WHITE;
				arc += 10;
			}
			if (!found)
				this.robot.getDrive().turnLeft(arc + 1);
		}
	}
	
	private static void showColorOnScreen(int colorID) {
		String color = "";
		switch(colorID) {
			case Color.RED:
				color = "Red";
				break;
			case Color.GREEN:
				color = "Green";
				break;
			case Color.BLUE:
				color = "Blue";
				break;
			case Color.YELLOW:
				color = "Yellow";
				break;
			case Color.BLACK:
				color = ""
		}
		
	}

}
