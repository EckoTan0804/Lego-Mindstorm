package mission;

import ev3Robot.Robot;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.robotics.Color;
import util.BrickScreen;

public class Labyrinth {

	private Robot robot;
	private boolean beginLabyrinth = false;
	private boolean endLabyrinth = false;

	public Labyrinth(Robot robot) {
		super();
		this.robot = robot;
	}

	public void startLabyrinth() {

		BrickScreen.show("Labyrinth");

		this.robot.changeSettingsForLabyrinth();

		try {
			Thread.sleep(20);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		float speed = 10f; // TODO: need to adjust the speed
		this.robot.getDrive().setMotorSpeed(speed, speed);
		this.robot.getDrive().goForwardWithMotors();

		int distance = 4;

		while (Button.LEFT.isUp() && (!endLabyrinth)) {

			BrickScreen.clearScreen();

			int colorID = this.getColorID();
			this.showColorIdOnScreen(colorID);

			switch (colorID) {

			case Color.RED:
				// BrickScreen.showStringOnScreen("Red", 0, 1);
				// this.robot.getDrive().flt();
				this.robot.getDrive().quickStop();
				this.robot.getDrive().turnRight(90);
				this.robot.getDrive().travel(distance);

				if (this.getColorID() != Color.WHITE) {
					this.findWay(distance);

					if (this.getColorID() != Color.WHITE) {
						this.findWay(distance);

						if (this.getColorID() != Color.WHITE) {
							this.findWay(distance);

							if (this.getColorID() != Color.WHITE) { /* dead end! */
								this.findWay(distance);
							}
						}
					}

				}
				break;

			case Color.BLACK: // TODO: need to measure the color ID of the ground

				/*
				 * The robot has drifted away from the white line
				 */
				// BrickScreen.showStringOnScreen("Black", 0, 1);
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
				// BrickScreen.showStringOnScreen("Blue", 0, 1);
				if (this.beginLabyrinth == false && this.endLabyrinth == false) {
					/* The robot arrives at the starting point */
					this.beginLabyrinth = true;
					this.robot.getDrive().travel(15);
					this.robot.getDrive().turnRight(90);
				} else if (this.beginLabyrinth == true && this.endLabyrinth == false) {
					/* The robot arrives at the destination */
					this.endLabyrinth = true;
				}
				break;

			case Color.WHITE:
				/*
				 * the robot runs on the white line, just go straight
				 */
				// BrickScreen.showStringOnScreen("White", 0, 1);
				this.robot.getDrive().goForwardWithMotors();
				break;

			default:
				break;

			}

		}
	}

	private void findLine() {
		boolean found = false;
		while (!found) {
			// this.robot.getDrive().travel(1);
			int arc = 0;
			while (arc < 90 && !found) {
				this.robot.getDrive().turnRight(10);
				found = this.robot.getColorID() == Color.WHITE;
				arc += 10;
			}
			if (!found)
				this.robot.getDrive().turnLeft(arc + 30);
		}
	}

	private void findWay(int distance) {
		this.robot.getDrive().travel(-distance);
		this.robot.getDrive().quickStop();
		this.robot.getDrive().turnLeft(90);
		this.robot.getDrive().travel(distance);
		this.robot.getDrive().quickStop();

	}

	private void showColorIdOnScreen(int colorID) {
		switch (colorID) {
		case Color.BLACK:
			BrickScreen.showStringOnScreen("Black", 0, 1);
			break;
		case Color.RED:
			BrickScreen.showStringOnScreen("Red", 0, 1);
			break;
		case Color.BLUE:
			BrickScreen.showStringOnScreen("Blue", 0, 1);
			break;
		case Color.WHITE:
			BrickScreen.showStringOnScreen("White", 0, 1);
			break;
		default:
			break;
		}
	}

	private int getColorID() {
		return (int) this.robot.getSensors().getColor();
	}

}
