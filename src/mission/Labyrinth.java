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

		this.robot.changeSettingsForLabyrinth();
		BrickScreen.show("Labyrinth");

		float speed = this.robot.getLeftMotor().getMaxSpeed(); // TODO: need to adjust the speed
		this.robot.getDrive().setMotorSpeed(speed, speed);
		this.robot.getDrive().goForwardWithMotors();

		while (Button.LEFT.isUp() && (!endLabyrinth)) {

			// int colorVal = (int) this.robot.getSensors().getColor();
			int colorID = this.robot.getColorID();
			BrickScreen.showStringOnScreen("color = " + colorID, 0, 1);
//			LCD.drawString("color = " + colorID, 0, 1);

			switch (colorID) {

			case Color.RED:
				/*
				 * the robot reaches a cross or a corner, needs to do a 90 degree rotation
				 */
				this.robot.getDrive().flt();
				this.robot.getDrive().turnRight(90);
				this.robot.getDrive().travel(5);

				if (this.robot.getColorID() != Color.WHITE) {

					if (this.robot.getColorID() == Color.BLACK) {
						this.findWay();
						
						if (this.robot.getColorID() != Color.WHITE) { 
							this.findWay();
							
							if (this.robot.getColorID() != Color.WHITE) { /* dead end! */
								this.findWay();
							}
						}
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
				if (this.beginLabyrinth == false && this.endLabyrinth == false) {
					/* The robot arrives at the starting point */
					this.beginLabyrinth = true;
				} else if (this.beginLabyrinth == true && this.endLabyrinth == false) {
					/* The robot arrives at the destination */
					this.endLabyrinth = true;
				}
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
	
	private void findWay() {
		this.robot.getDrive().travel(-5);
		this.robot.getDrive().turnLeft(90);
		this.robot.getDrive().travel(5);
	}
	


}
