package ev3Robot;

import lejos.robotics.RegulatedMotor;
import lejos.robotics.navigation.DifferentialPilot;

public class Drive {

	private DifferentialPilot pilot;
	private Robot robot;

	public Drive(double wheelDiameter, double trackWidth, RegulatedMotor leftMotor, RegulatedMotor rightMotor) {
		this.pilot = new DifferentialPilot(wheelDiameter, trackWidth, leftMotor, rightMotor);
	}

	public Drive(Robot robot) {
		this.robot = robot;
		this.pilot = new DifferentialPilot(robot.getWHEEL_DIAMETER(), robot.getTRACK_WIDTH(), robot.getLeftMotor(),
				robot.getRightMotor());
	}

	public void setSpeed(double speed) {
		this.pilot.setTravelSpeed(speed);
	}

	public void goForwardWithPilot() {
		this.pilot.forward();
	}

	public void goBackwardWithPilot() {
		this.pilot.backward();
	}

	public void goForwardWithMotors() {
		this.robot.getLeftMotor().startSynchronization();
		this.robot.getLeftMotor().forward();
		this.robot.getRightMotor().forward();
		this.robot.getLeftMotor().endSynchronization();
	}

	public void goBackwardWithMotors() {
		this.robot.getLeftMotor().startSynchronization();
		this.robot.getLeftMotor().backward();
		this.robot.getRightMotor().backward();
		this.robot.getLeftMotor().endSynchronization();
	}

	public void setMotorSpeed(float leftMotorTargetSpeed, float rightMotorTargetSpeed) {
		this.robot.getLeftMotor().startSynchronization();
		this.robot.getLeftMotor().setSpeed(leftMotorTargetSpeed);
		this.robot.getRightMotor().setSpeed(rightMotorTargetSpeed);
		this.robot.getLeftMotor().endSynchronization();
	}
	
	public void setLeftMotorSpeed(float targetSpeed) {
		this.robot.getLeftMotor().setSpeed(targetSpeed);
	}
	
	public void setRightMotorSpeed(float targetSpeed) {
		this.robot.getRightMotor().setSpeed(targetSpeed);
	}

	/**
	 * Moves the robot a specific distance (unit: cm) in an (hopefully) straight
	 * line.
	 * 
	 * @param distance
	 *            A positive distance causes forward motion, a negative distance
	 *            moves backward.
	 */
	public void travel(double distance) {
		this.pilot.travel(distance);
	}

	/**
	 * Moves the robot a specified distance along an arc of specified radius, after
	 * which the robot stops moving. The units (inches, cm) for distance must be the
	 * same as the units used for radius.
	 * 
	 * @param radius
	 *            If radius is positive, the robot arcs left, and the center of the
	 *            turning circle is on the left side of the robot. If radius is
	 *            negative, the robot arcs right, and the center of the turning
	 *            circle is on the right side of the robot. If radius is zero, the
	 *            robot rotates in place
	 * @param distance
	 *            The robot will stop when it has moved along the arc distance
	 *            units. If distance is positive, the robot will move travel
	 *            forwards. If distance is negative, the robot will move travel
	 *            backwards. If distance is zero, the robot will not move and the
	 *            method returns immediately.
	 */
	public void travelArc(double radius, double distance) {
		this.pilot.travelArc(radius, distance, false);
	}

	public boolean isMoving() {
		return this.pilot.isMoving();
	}

	public void stopWithPilot() {
		this.pilot.stop();
	}

	public void stopWithMotors() {
		this.robot.getLeftMotor().startSynchronization();
		this.robot.getLeftMotor().stop();
		this.robot.getRightMotor().stop();
		this.robot.getLeftMotor().endSynchronization();
	}

	/**
	 * 
	 * @param angle
	 *            The wanted angle of rotation in degrees. Positive angle rotate
	 *            left (anti-clockwise), negative right.
	 */
	public void rotate(double angle) {
		this.pilot.rotate(angle, false);
	}
	
	public void turnLeft(double angle) throws IllegalArgumentException {
		if (angle < 0 || angle > 180) {
			throw new IllegalArgumentException();
		}
		this.rotate(angle);
	}
	
	public void turnRight(double angle) throws IllegalArgumentException {
		if (angle < 0 || angle > 180) {
			throw new IllegalArgumentException();
		}
		this.rotate(-angle);
	}
	
	public void flt() {
		this.robot.getLeftMotor().flt();
		this.robot.getRightMotor().flt();
	}
	
	

}
