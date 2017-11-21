package ev3Robot;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.navigation.DifferentialPilot;
import mission.LineFollower;
import mission.Mission;
import mission.MissionMenu;
import sensor.SensorThread;
import sensor.SingleValueSensorWrapper;

public class Robot implements Runnable {

	// Date need to be measured and adjusted
	private final double WHEEL_DIAMETER = 3.2d; // unit:cm
	private final double TRACK_WIDTH = 11d; // unit: cm
	private final Port LEFT_MOTOR = MotorPort.A;
	private final Port RIGHT_MOTOR = MotorPort.D;

	private static final Port COLOR_SENSOR = SensorPort.S1;
	private static final Port TOUCH_SENSOR_1 = SensorPort.S2;
	private static final Port TOUCH_SENSOR_2 = SensorPort.S3;
	private static final Port ULTRASONIC_SENSOR = SensorPort.S4;

	// private DifferentialPilot pilot;
	private SensorThread sensors;
	private MissionMenu missionMenu;

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	private Drive drive;

	public Robot() {

		this.leftMotor = new EV3LargeRegulatedMotor(LEFT_MOTOR);
		this.rightMotor = new EV3LargeRegulatedMotor(RIGHT_MOTOR);
		// this.pilot = new DifferentialPilot(WHEEL_DIAMETER, TRACK_WIDTH, leftMotor,
		// rightMotor, false);

		EV3ColorSensor colorS = new EV3ColorSensor(COLOR_SENSOR);
		EV3TouchSensor touchS1 = new EV3TouchSensor(TOUCH_SENSOR_1);
		EV3TouchSensor touchS2 = new EV3TouchSensor(TOUCH_SENSOR_2);
		EV3UltrasonicSensor ultraS = new EV3UltrasonicSensor(ULTRASONIC_SENSOR);

		SingleValueSensorWrapper color = new SingleValueSensorWrapper(colorS, "Red");
		SingleValueSensorWrapper touch1 = new SingleValueSensorWrapper(touchS1, "Touch");
		SingleValueSensorWrapper touch2 = new SingleValueSensorWrapper(touchS2, "Touch");
		SingleValueSensorWrapper distance = new SingleValueSensorWrapper(ultraS, "Distance");

		this.sensors = new SensorThread(color, touch1, touch2, distance);
		this.missionMenu = new MissionMenu();
		this.drive = new Drive(this);

		new Thread(this.sensors).start();

	}

	public void mainLoop() throws InterruptedException {
		// new Thread(this.sensors).start();
		new Thread(this).start(); // start the program
		Button.LEDPattern(1); // green light
		this.missionMenu.startGUI(this); // show menu on the brick's screen
		System.exit(0);
	}

	@Override
	public void run() {
		// show mission menu on the brick's screen
		// this.missionMenu = new MissionMenu();

	}

	// private void sleep(int millis) {
	// try {
	// Thread.sleep(millis);
	// } catch (InterruptedException e) {
	// e.printStackTrace();
	// }
	// }

	// ============= setters and getters ========================

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

	public EV3LargeRegulatedMotor getLeftMotor() {
		return leftMotor;
	}

	public void setLeftMotor(EV3LargeRegulatedMotor leftMotor) {
		this.leftMotor = leftMotor;
	}

	public EV3LargeRegulatedMotor getRightMotor() {
		return rightMotor;
	}

	public void setRightMotor(EV3LargeRegulatedMotor rightMotor) {
		this.rightMotor = rightMotor;
	}

	public double getWHEEL_DIAMETER() {
		return WHEEL_DIAMETER;
	}

	public double getTRACK_WIDTH() {
		return TRACK_WIDTH;
	}

	public Drive getDrive() {
		return drive;
	}

	public void setDrive(Drive drive) {
		this.drive = drive;
	}

}
