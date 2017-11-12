package mission;

import ev3Robot.Robot;
import lejos.hardware.Button;

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

	private float offset = (WHITE - BLACK) / 2;

	/*
	 * Target power level, power level of both motors when the robot is supposed to
	 * go straight ahead, controls how fast the robot is moving along the line
	 */
	private final float Tp = 50;

	/*
	 * the constant for the Proportional controller, controls how fast the
	 * controllers will try to get back to the line edge when it has drifted away
	 * from it
	 */
	private final float Kp = (float) ((Tp - 0) / (0.14 - 0));

	/*
	 * the constant for the Integral controller
	 */
	private final float Ki = 1f;

	/*
	 * the constant for the derivative controller
	 */
	private final float Kd = 100f;

	private float integral = 0f;
	private float lastError = 0f;
	private float derivative = 0f;

	private Robot robot;

	public LineFollowerThread(Robot robot) {
		super();
		this.robot = robot;
	}

	@Override
	public void run() {
		this.lineFollowing();
	}

	private void lineFollowing() {

		this.robot.getDrive().setMotorSpeed(Tp, Tp);
		this.robot.getDrive().goForwardWithMotors();

		while (Button.LEFT.isUp()) {

			// get the sample value measured by color sensor
			float sampleVal = this.robot.getSensors().getColor();

			// calculate turn, based on this value
			float error = sampleVal - offset;
			integral = integral + error;
			derivative = error - lastError;
			float turn = Kp * error + Ki * integral + Kd * derivative;

			// adjust the power of left and right motors in order to make the robot follow
			// the line
			this.robot.getDrive().setMotorSpeed(this.robot.getLeftMotor().getTachoCount() - turn,
					this.robot.getRightMotor().getTachoCount() + turn);

			// robot goes forward
			this.robot.getDrive().goForwardWithMotors();
			
			// update error
			this.lastError = error;

			try {
				Thread.sleep(20);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}

	}

}
