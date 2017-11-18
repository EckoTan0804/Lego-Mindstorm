package mission;

import ev3Robot.Drive;
import ev3Robot.Robot;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;

/**
 * For mission Line-Following a PID-controller will be used.
 * <p>
 * Reference: "PID Controller For Lego Mindstorms Robots"
 * </p>
 * 
 * @author EckoTan
 *
 */
public class LineFollowerThread implements Runnable {

	private final float BLACK = 0.05f;
	private final float WHITE = 0.33f;
	private final float EPS = 0.01f;

	private float offset = (WHITE + BLACK) / 2;

	/*
	 * Target power level, power level of both mostors when the robot is supposed to
	 * go straight ahead, controls how fast the robot is moving along the line
	 */
	private final float Tp = 220f;

	/*
	 * the constant for the Proportional controller, controls how fast the
	 * controllers will try to get back to the line edge when it has drifted away
	 * from it
	 */
	private final float Kp = (float) (((Tp - 0) / (WHITE - offset)) * 1.8);

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
	private Drive drive;

	public LineFollowerThread(Robot robot) {
		super();
		this.robot = robot;
		this.drive = robot.getDrive();
	}

	@Override
	public void run() {
		this.lineFollowing();
	}

	private void lineFollowing() {

		Sound.beep();

		this.robot.getDrive().setMotorSpeed(Tp, Tp);
		this.robot.getDrive().goForwardWithMotors();

		/**
		 * use a timer to count how many times the color sensor detects BLACK after
		 * detecting WHITE
		 */
		int timer = 0;

		boolean shouldTurnLeftLookForLine = true;

		while (Button.LEFT.isUp()) { // stop the routine and back to the main menu if LEFT is pressed

			LCD.clear();
			LCD.drawString(Mission.LINE_FOLLOWING.getMission(), 0, 0);

			float leftTargetSpeed = 0;
			float rightTargetSpeed = 0;
			float error = 0;

			// get the real time sample value measured by color sensor
			float sampleVal = this.robot.getSensors().getColor();
			LCD.drawString("color sensor val = " + sampleVal, 0, 1);

			if (sampleVal >= offset) { // the color sensor sees WHITE
				timer = 0;
			} else { // the color sensor sees BLACK
				timer++;
			}

			if (sampleVal > WHITE - EPS) { // special case: the robot need to do a 90 degree rotation
				this.robot.getDrive().stopWithMotors();
				leftTargetSpeed = -Tp;
				rightTargetSpeed = Tp;

			} else if (timer >= 3) { // special case: the robot reaches a line gap

				this.robot.getDrive().stopWithMotors();

				// Idea: let the robot move forward like a "s" to look for the line
				if (shouldTurnLeftLookForLine) {
					this.robot.getDrive().travelArc(2, 4); // TODO: parameters need to be measured and adjusted
					shouldTurnLeftLookForLine = false;
				} else {
					this.robot.getDrive().travelArc(-2, 2);
					shouldTurnLeftLookForLine = true;
				}

			} else { // normal case

				// calculate turn, based on the sample value
				error = sampleVal - offset;
				integral = integral + error;
				derivative = error - lastError;
				// float turn = Kp * error + Ki * integral + Kd * derivative;
				float turn = Kp * error; // only a P-controller will be used.

				// adjust the power of left and right motors in order to make the robot follow
				// the line
				leftTargetSpeed = Tp - turn;
				rightTargetSpeed = Tp + turn;
			}

			// print the target speed of left and right motors on the brick's screen
			LCD.drawString("Left Motor = " + leftTargetSpeed, 0, 2);
			LCD.drawString("Right Motor = " + rightTargetSpeed, 0, 3);

			// adjust the robot's movement in order to make the robot follow the line
			this.adjustRobotMovement(this.robot, leftTargetSpeed, rightTargetSpeed);

			// this.robot.getLeftMotor().startSynchronization();
			// this.robot.getRightMotor().startSynchronization();
			// if (leftTargetSpeed < 0) {
			// this.robot.getDrive().setLeftMotorSpeed(-leftTargetSpeed);
			// this.robot.getLeftMotor().backward();
			// } else {
			// this.robot.getDrive().setLeftMotorSpeed(leftTargetSpeed);
			// this.robot.getLeftMotor().forward();
			// }
			// if (rightTargetSpeed < 0) {
			// this.robot.getDrive().setRightMotorSpeed(-rightTargetSpeed);
			// this.robot.getRightMotor().backward();
			// } else {
			// this.robot.getDrive().setRightMotorSpeed(rightTargetSpeed);
			// this.robot.getRightMotor().forward();
			// }
			// this.robot.getLeftMotor().endSynchronization();
			// this.robot.getRightMotor().endSynchronization();

			// update error
			this.lastError = error;

			try {
				Thread.sleep(20);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}

		Sound.beepSequence();
		this.drive.stopWithMotors();
	}

	private void adjustRobotMovement(Robot robot, float leftTargetSpeed, float rightTargetSpeed) {

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
