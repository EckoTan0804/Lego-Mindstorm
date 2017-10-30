package ev3Robot;

import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.navigation.DifferentialPilot;
import mission.MissionMenu;

public class Robot implements Runnable {

	// Date need to be measured and adjusted
	private static final double WHEEL_DIAMETER = 20d;
	private static final double TRACK_WIDTH = 20d;
	private static final Port LEFT_MOTOR = MotorPort.A;
	private static final Port RIGHT_MOTOR = MotorPort.D;

	private static final Port COLOR_SENSOR = SensorPort.S1;
	private static final Port TOUCH_SENSOR_1 = SensorPort.S2;
	private static final Port TOUCH_SENSOR_2 = SensorPort.S3;
	private static final Port ULTRASONIC_SENSOR = SensorPort.S4;

	private DifferentialPilot pilot;
	private SensorThread sensors;
	private MissionMenu missionMenu;

	public Robot() {

		EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LEFT_MOTOR);
		EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(RIGHT_MOTOR);
		this.pilot = new DifferentialPilot(WHEEL_DIAMETER, TRACK_WIDTH, leftMotor, rightMotor, false);

		EV3ColorSensor colorS = new EV3ColorSensor(COLOR_SENSOR);
		EV3TouchSensor touchS1 = new EV3TouchSensor(TOUCH_SENSOR_1);
		EV3TouchSensor touchS2 = new EV3TouchSensor(TOUCH_SENSOR_2);
		EV3UltrasonicSensor ultraS = new EV3UltrasonicSensor(ULTRASONIC_SENSOR);

		SingleValueSensorWrapper color = new SingleValueSensorWrapper(colorS, "Red");
		SingleValueSensorWrapper touch1 = new SingleValueSensorWrapper(touchS1, "Touch");
		SingleValueSensorWrapper touch2 = new SingleValueSensorWrapper(touchS2, "Touch");
		SingleValueSensorWrapper distance = new SingleValueSensorWrapper(ultraS, "Distance");

		this.sensors = new SensorThread(color, touch1, touch2, distance);

	}

	public void mainLoop() throws InterruptedException {
		new Thread(this.sensors).start();
		new Thread(this).start(); // start the program
		Button.LEDPattern(1); // green light

		while (Button.ENTER.isUp()) { // manual stop the program when Button "Enter" is pressed
			
			// choose mission-routine from mission menu
			switch (this.missionMenu.select()) {
			case MissionMenu.MENU_ITEM_LINE_FOLLOWING:
				// line following routine will be executed
				break;
			case MissionMenu.MENU_ITEM_LABYRINTH:
				// labyrinth routine will be executed
				break;
			case MissionMenu.MENU_ITEM_OBSTACLE_SHIFTING:
				// obstacle shifting routine will be executed
				break;
			default:
				// bridge routine will be executed
				break;
			}

			this.sleep(20);
		}
		System.exit(0);
	}

	public void goForward() {
		this.pilot.forward();
	}

	public void goBackward() {
		this.pilot.backward();
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
		this.pilot.travel(distance, true);
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
		this.pilot.travelArc(radius, distance, true);
	}

	public boolean isMoving() {
		return this.pilot.isMoving();
	}

	public void stop() {
		this.pilot.stop();
	}

	/**
	 * 
	 * @param angle
	 *            The wanted angle of rotation in degrees. Positive angle rotate
	 *            left (anti-clockwise), negative right.
	 */
	public void rotate(double angle) {
		this.pilot.rotate(angle, true);
	}

	public DifferentialPilot getPilot() {
		return pilot;
	}

	public void setPilot(DifferentialPilot pilot) {
		this.pilot = pilot;
	}

	private void sleep(int millis) {
		try {
			Thread.sleep(millis);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

	@Override
	public void run() {
		// show mission menu on the brick's screen
		this.missionMenu = new MissionMenu();

	}

	public SensorThread getSensors() {
		return sensors;
	}

	public void setSensors(SensorThread sensors) {
		this.sensors = sensors;
	}

	public MissionMenu getMissionMenu() {
		return missionMenu;
	}

	public void setMissionMenu(MissionMenu missionMenu) {
		this.missionMenu = missionMenu;
	}
	
	

}
