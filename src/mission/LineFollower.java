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

	private final float BLACK = 0.05f;
	private final float WHITE = 0.33f;
	private final float EPS = 0.01f;

	private float offset = (WHITE + BLACK) / 2;

	/*
	 * Target power level, power level of both mostors when the robot is supposed to
	 * go straight ahead, controls how fast the robot is moving along the line
	 */
	private final float Tp = 250f;

	/*
	 * the constant for the Proportional controller, controls how fast the
	 * controllers will try to get back to the line edge when it has drifted away
	 * from it
	 */
	private final float Kp = (float) (((Tp - 0) / (WHITE - offset)) * 1.2);

	/*
	 * the constant for the Integral controller
	 */
	private final float Ki = 1f;

	/*
	 * the constant for the derivative controller
	 */
	private final float Kd = 50f;

	// /*
	// * Target power level, power level of both motors when the robot is supposed
	// to
	// * go straight ahead, controls how fast the robot is moving along the line
	// */
	// private final float Tp = 400;

	private float integral = 0f;
	private float lastError = 0f;
	private float derivative = 0f;

	private Robot robot;
	// private Drive drive;

	public LineFollower(Robot robot) {

		this.robot = robot;
		// this.drive = robot.getDrive();
	}

	public void startLineFollowing() {

		Sound.beep();

		this.robot.getDrive().setMotorSpeed(Tp, Tp);
		this.robot.getDrive().goForwardWithMotors();

		while (Button.LEFT.isUp()) { /* stop the routine and back to the main menu if LEFT is pressed */

			BrickScreen.clearScreen();
			BrickScreen.show(Mission.LINE_FOLLOWING.getMission());
			// LCD.clear();
			// LCD.drawString(Mission.LINE_FOLLOWING.getMission(), 0, 0);

			float leftTargetSpeed = 0;
			float rightTargetSpeed = 0;
			float error = 0;

			/* get the real time sample value measured by color sensor */

			float sampleVal = this.robot.getSensors().getColor();
			BrickScreen.show("color = " + sampleVal);
			BrickScreen.show("TS1 = " + this.robot.getSensors().getTouch1());
			BrickScreen.show("TS2 = " + this.robot.getSensors().getTouch2());
			// LCD.drawString("color = " + sampleVal, 0, 1);
			// LCD.drawString("TS1 = " + this.robot.getSensors().getTouch1(), 0, 5);
			// LCD.drawString("TS2 = " + this.robot.getSensors().getTouch2(), 0, 6);

			if ((this.robot.getSensors().getTouch1()) > 0.2 && (this.robot.getSensors().getTouch2()) > 0.2) {
				/* special case: the robot reaches an obstacle */
				overObstacle();

			} else if (sampleVal > WHITE - EPS) { /* special case: the robot need to do a 90 degree rotation */

				// this.robot.getDrive().stopWithMotors();
				leftTargetSpeed = -Tp;
				rightTargetSpeed = Tp;

				/* adjust the robot's movement in order to make the robot follow the line */
				this.adjustRobotMovement(this.robot, leftTargetSpeed, rightTargetSpeed);

			} else if (sampleVal < BLACK + EPS) { /* special case: the robot reaches a line gap */

				// this.robot.getDrive().stopWithMotors();

				int arc = 0;
				boolean found = false;
				while (arc < 90 && !found) {
					this.robot.getDrive().turnRight(10);
					found = this.robot.getSensors().getColor() >= offset;
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
				this.adjustRobotMovement(this.robot, leftTargetSpeed, rightTargetSpeed);
			}

			/* update error */
			this.lastError = error;

			try {
				Thread.sleep(20);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}

		Sound.beepSequence();
		LCD.clear();
		this.robot.getDrive().stopWithMotors();
	}

	private void overObstacle() {
		this.robot.getDrive().travel(-3);
		this.robot.getDrive().turnRight(90);
		this.robot.getDrive().travel(20);
		this.robot.getDrive().turnLeft(90);
		this.robot.getDrive().travel(40);
		this.robot.getDrive().turnLeft(90);
		this.robot.getDrive().travel(22);
		this.robot.getDrive().turnRight(90);
	}

	private void findLine() {
		boolean found = false;
		while (!found) {
			this.robot.getDrive().travel(7);
			int arc = 0;
			while (arc < 90 && !found) {
				this.robot.getDrive().turnRight(10);
				found = this.robot.getSensors().getColor() > BLACK + 2 * EPS;
				arc += 10;
			}
			if (!found)
				this.robot.getDrive().turnLeft(arc + 1);
		}
	}

	private void adjustRobotMovement(Robot robot, float leftTargetSpeed, float rightTargetSpeed) {

		/* print the target speed of left and right motors on the brick's screen */
		BrickScreen.show("L= " + leftTargetSpeed);
		BrickScreen.show("L= " + rightTargetSpeed);
//		LCD.drawString("L= " + leftTargetSpeed, 0, 2);
//		LCD.drawString("R= " + rightTargetSpeed, 0, 3);

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
}
