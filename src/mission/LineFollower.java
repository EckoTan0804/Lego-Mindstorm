package mission;

import ev3Robot.Drive;
import ev3Robot.Robot;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import util.BrickScreen;

/**
 * For mission Line-Following a PID-controller will be used.
 * <p>
 * Reference: "PID Controller For Lego Mindstorms Robots"
 * </p>
 * 
 * @author EckoTan
 *
 */
public class LineFollower {

	private boolean endLineFollower = false;

	/**
	 * <li>true, if the color sensor is in "Red" mode <br>
	 * <li>false, if the color sensor is in "RGB" mode
	 */
	private boolean useRedMode = true;

	private final float BLACK = 0.05f;
	private final float WHITE = 0.33f;
	private final float EPS = 0.01f;

	private float offset = (WHITE + BLACK) / 2;

	/*
	 * Target power level, power level of both motors when the robot is supposed to
	 * go straight ahead, controls how fast the robot is moving along the line
	 */
	private final float Tp = 220f;

	/*
	 * the constant for the Proportional controller, controls how fast the
	 * controllers will try to get back to the line edge when it has drifted away
	 * from it
	 */
	private final float Kp = (((Tp - 0) / (WHITE - offset)) * 1.2f);

	/*
	 * the constant for the Integral controller
	 */
	private final float Ki = 1f;

	/*
	 * the constant for the derivative controller
	 */
	private final float Kd = 50f;

	private float integral = 0f;
	private float lastError = 0f;
	private float derivative = 0f;

	private Robot robot;

	public LineFollower(Robot robot) {
		this.robot = robot;
	}

	public void startLineFollowing() {

		Sound.beep();

		this.robot.changeSettingsForLineFollower();

		this.robot.getDrive().setMotorSpeed(Tp, Tp);
		this.robot.getDrive().goForwardWithMotors();

		while (Button.LEFT.isUp() && !this.endLineFollower) { /*
																 * stop the routine and back to the main menu if LEFT is
																 * pressed or endLineFollower is true
																 */

			float leftTargetSpeed = 0;
			float rightTargetSpeed = 0;

			BrickScreen.clearScreen();
			BrickScreen.show(Mission.LINE_FOLLOWING.getMission());

			if (useRedMode) { /* The color sensor use "RED" mode to follow line */

				float error = 0;

				/* get the real-time sample value measured by color sensor */

				float sampleVal = this.robot.getSensors().getColor();
				BrickScreen.show("color = " + sampleVal);
				BrickScreen.show("TS1 = " + this.robot.getSensors().getTouch1());
				BrickScreen.show("TS2 = " + this.robot.getSensors().getTouch2());

				if ((this.robot.getSensors().getTouch1()) > 0.2 && (this.robot.getSensors().getTouch2()) > 0.2) {
					/* special case: the robot reaches an obstacle */
					overObstacle();

				} else if (sampleVal > WHITE - 2 * EPS) { /* special case: the robot need to do a 90 degree rotation */

					// this.robot.getDrive().stopWithMotors();
					leftTargetSpeed = -1.2f * Tp;
					rightTargetSpeed = 1.2f * Tp;

					/* adjust the robot's movement in order to make the robot follow the line */
					this.robot.getDrive().adjustRobotMovement(leftTargetSpeed, rightTargetSpeed);

				} else if (sampleVal < BLACK + EPS) { /* special case: the robot reaches a line gap */

					int arc = 0;
					boolean found = false;
					while (arc < 90 && !found) {
						this.robot.getDrive().turnRight(10);
						found = this.robot.getSensors().getColor() >= offset + 2 * EPS;
						arc += 10;
					}

					if (!found) {
						this.robot.getDrive().turnLeft(arc + 10);
						findLine();
					}
 
				} else { /* normal case */

					/* calculate turn, based on the sample value */
					error = sampleVal - offset;
					integral = integral + error;
					derivative = error - lastError;
					// float turn = Kp * error + Ki * integral + Kd * derivative;
					float turn = Kp * error; /* only a P-controller will be used. */

					/*
					 * adjust the power of left and right motors in order to make the robot follow
					 * the line
					 */
					leftTargetSpeed = Tp - turn;
					rightTargetSpeed = Tp + turn;

					/* adjust the robot's movement in order to make the robot follow the line */
					this.robot.getDrive().adjustRobotMovement(leftTargetSpeed, rightTargetSpeed);
				}

				/* update error */
				this.lastError = error;

			} else { /* The color sensor use "RGB" mode to follow line */

				float[] rgb = this.getColorArrayInRgbMode();

				float red = rgb[0];
				float green = rgb[1];
				float blue = rgb[2];

				BrickScreen.show("r: " + red);
				BrickScreen.show("g: " + green);
				BrickScreen.show("b: " + blue);

				if (red > 15) {
					if (red > 23 && green < 35 && blue < 23) { // red
						BrickScreen.show("Red");
						this.endLineFollower = true;
					} else { // white
						rightTargetSpeed = 1.3f * Tp;
						leftTargetSpeed = 1.0f * Tp;
						BrickScreen.show("White");
						this.robot.getDrive().adjustRobotMovement(leftTargetSpeed, rightTargetSpeed);
					}
				} else { // blue
					if (blue > 16 && green <= 23) {
						/* Nothing to do with blue, just ignore it. */
					} else { // black
						rightTargetSpeed = (float) (-0.5) * Tp;
						leftTargetSpeed = (float) 1.1f * Tp;
						this.robot.getDrive().adjustRobotMovement(leftTargetSpeed, rightTargetSpeed);
						BrickScreen.show("Black");
					}
				}
			}

			try {
				Thread.sleep(20);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}

		}

		/*
		 * Use a beep sequence to mark that the Line Follower routine is already
		 * finished.
		 */
		Sound.beepSequenceUp();
		
//		this.fromLineFollowerToLabyrinth();

	}

	private void overObstacle() {
		Sound.buzz();
		
		this.robot.getDrive().travel(-3);
		this.robot.getDrive().turnRight(90);
		this.robot.getDrive().travel(26);
		this.robot.getDrive().turnLeft(90);
		this.robot.getDrive().travel(36);
		this.robot.getDrive().turnLeft(90);
		this.robot.getDrive().travel(22);
		// this.robot.getDrive().turnRight(90);

		this.robot.changeSettingsForLabyrinth();
		this.useRedMode = false;
		
		Sound.twoBeeps();
		
	}

	private void findLine() {
		boolean found = false;
		while (!found) {
			this.robot.getDrive().travel(9);
			int arc = 0;
			while (arc < 90 && !found) {
				this.robot.getDrive().turnRight(10);
				found = this.robot.getSensors().getColor() > BLACK + 2 * EPS;
				arc += 10;
			}
			if (!found)
				 //this.robot.getDrive().turnLeft(arc + 1);
				this.robot.getDrive().turnLeft(arc);
		}
	}

	public void fromLineFollowerToLabyrinth() {
		
		this.robot.getDrive().travel(2);

		BrickScreen.clearScreen();

		float speed = 300f;

		while (Button.LEFT.isUp()) {

			BrickScreen.clearScreen();

			float[] rgb = this.getColorArrayInRgbMode();

			float red = rgb[0];
			float green = rgb[1];
			float blue = rgb[2];

			BrickScreen.show("r: " + red);
			BrickScreen.show("g: " + green);
			BrickScreen.show("b: " + blue);
			
			if (this.isBlueInRgbMode(rgb)) 
				break;

			/* The standard distance to the wall is about 17.4 cm. */
			float dist = this.robot.getSensors().getDistance() * 100;
			BrickScreen.show("Dist: " + dist + " cm");

			float rightTargetSpeed = 0f;
			float leftTargetSpeed = 0f;

			if (dist > 30) {
				rightTargetSpeed = speed;
				leftTargetSpeed = speed;
			} else if (dist > 18) {
				
				/* turn right */
				rightTargetSpeed = speed;
				leftTargetSpeed = 1.1f * speed;

			} else if (dist < 17) {
				
				/* turn left */
				rightTargetSpeed = 1.1f * speed;
				leftTargetSpeed = speed;
				
			} else {
				rightTargetSpeed = speed;
				leftTargetSpeed = speed;
			}

			this.robot.getDrive().adjustRobotMovement(leftTargetSpeed, rightTargetSpeed);
		}
		
		this.robot.getDrive().stopWithMotors();

	}

	private float[] getColorArrayInRgbMode() {
		float[] rgb = this.robot.getSensors().getColorArray();
		for (int i = 0; i < rgb.length; i++) {
			rgb[i] *= 255;
		}
		return rgb;
	}

	private boolean isBlueInRgbMode(float[] colorArray) {
		return colorArray[0] <= 15 && colorArray[1] <= 23 && colorArray[2] > 16;
		//return colorArray[2] >= 10;
	}
}
