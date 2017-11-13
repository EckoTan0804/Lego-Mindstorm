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
	private final float Tp = 200f;

	/*
	 * the constant for the Proportional controller, controls how fast the
	 * controllers will try to get back to the line edge when it has drifted away
	 * from it
	 */
	private final float Kp = (float) ((Tp - 0)/ (WHITE - offset)) * 2;

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

		while (Button.LEFT.isUp()) { // stop the routine and back to the main menu if LEFT is pressed

			// get the sample value measured by color sensor
			// this.robot.
			// this.drive.goForwardWithMotors();
				float leftTargetSpeed = 0;
				float rightTargetSpeed = 0;
				float error = 0;
				float sampleVal = this.robot.getSensors().getColor();
				
				LCD.clear();
				LCD.drawString(Mission.LINE_FOLLOWING.getMission(), 0, 0);
				LCD.drawString("val = " + sampleVal, 0, 1);
				//if (sampleVal < BLACK + EPS) {
				//	leftTargetSpeed = Tp;
				//	rightTargetSpeed = Tp;
				//} else 
				if (sampleVal > WHITE - EPS) {
					this.robot.getDrive().stopWithMotors();
					leftTargetSpeed = -Tp;
					rightTargetSpeed = Tp;
				} else {
					// calculate turn, based on this value
					error = sampleVal - offset;
					integral = integral + error;
					derivative = error - lastError;
					float turn = Kp * error + Ki * integral + Kd * derivative;
					//float turn = Kp * error;

					// adjust the power of left and right motors in order to make the robot follow
					// the line
					leftTargetSpeed = Tp - turn;
					rightTargetSpeed = Tp + turn;
				}
				LCD.drawString("L = " + leftTargetSpeed, 0, 2);
				LCD.drawString("R = " + rightTargetSpeed, 0, 3);
				//this.robot.getDrive().setMotorSpeed(leftTargetSpeed,
				//		rightTargetSpeed);
				
				this.robot.getLeftMotor().startSynchronization();
				this.robot.getRightMotor().startSynchronization();
				if (leftTargetSpeed < 0) {
					this.robot.getDrive().setLeftMotorSpeed(-leftTargetSpeed);
					this.robot.getLeftMotor().backward();
				} else {
					this.robot.getDrive().setLeftMotorSpeed(leftTargetSpeed);
					this.robot.getLeftMotor().forward();
				}
				if (rightTargetSpeed < 0) {
					this.robot.getDrive().setRightMotorSpeed(-rightTargetSpeed);
					this.robot.getRightMotor().backward();
				} else {
					this.robot.getDrive().setRightMotorSpeed(rightTargetSpeed);
					this.robot.getRightMotor().forward();
				}
				this.robot.getLeftMotor().endSynchronization();
				this.robot.getRightMotor().endSynchronization();
				

				// robot goes forward
//				this.robot.getDrive().goForwardWithMotors();

				

				// this.drive.setMotorSpeed(Tp + turn, Tp - turn);
				// this.drive.goForwardWithMotors();
				//// this.robot.goForward();

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
	
//	private boolean isGeneral(float sampleVal) {
//		int rotateAngle = 0;
//		if (sampleVal <= BLACK) {
//			this.robot.getDrive().stopWithMotors();
//			
//		}
//	}
}
