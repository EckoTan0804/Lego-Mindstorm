package mission;

import ev3Robot.Drive;
import ev3Robot.Robot;
import lejos.hardware.Button;
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

	private float offset = WHITE - BLACK;

	/*
	 * the constant for the Proportional controller, controls how fast the
	 * controllers will try to get back to the line edge when it has drifted away
	 * from it
	 */
	private final float Kp = 100f;

	/*
	 * the constant for the Integral controller
	 */
	private final float Ki = 1f;

	/*
	 * the constant for the derivative controller
	 */
	private final float Kd = 50f;

	/*
	 * Target power level, power level of both motors when the robot is supposed to
	 * go straight ahead, controls how fast the robot is moving along the line
	 */
	private final float Tp = 400;

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
		LCD.drawString("HELLO", 0, 1);
		this.lineFollowing();
	}

	private void lineFollowing() {
//		this.robot.
//		this.drive.goForwardWithMotors();
		while (Button.LEFT.isUp()) {
			float sampleVal = this.robot.getSensors().getColor();
			float error = sampleVal - offset;
			integral = integral + error;
			derivative = error - lastError;
			float turn = Kp * error + Ki * integral + Kd * derivative;
			this.drive.setMotorSpeed(Tp + turn, Tp - turn);
			this.drive.goForwardWithMotors();
//			this.robot.goForward();
			this.lastError = error;
		}
		this.drive.stopWithMotors();

	}

}
