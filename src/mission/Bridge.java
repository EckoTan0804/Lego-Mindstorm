package mission;

import ev3Robot.Robot;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import util.BrickScreen;
import lejos.hardware.sensor.EV3UltrasonicSensor;

public class Bridge {
	
	private final int RED = 0;
	private final int GREEN = 1;
	private final int BLUE = 2;
	private final float Tp1 = 250f;
	private final float Tp2 = 300f;
	private final float disToGround = 0.12f;
	
	
	private Robot robot;
	
	

	public Bridge(Robot robot) {
		super();
		this.robot = robot;
	}

	public void startBridge() {

		BrickScreen.clearScreen();

		this.robot.changeSettingsForLabyrinth();
		
		this.robot.getMediumMotor().rotate(-75);

		try {
			Thread.sleep(20); 
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		boolean flag = false;
		
		float speed = Tp1; // TODO: need to adjust the speed
		this.robot.getDrive().setMotorSpeed(speed, speed);
		this.robot.getDrive().goForwardWithMotors();
		float leftTargetSpeed = 0f;
		float rightTargetSpeed = 0f;
		
		while (Button.LEFT.isUp()) {
			
			BrickScreen.clearScreen();
			
			float[] rgb = this.robot.getSensors().getColorArray();
			float blue = rgb[BLUE] * 255;
			
			BrickScreen.show("b: " + blue);
			
			if (blue >= 10) {
				flag = true;
				break;
			}	
			
			float dis = this.robot.getSensors().getDistance();
			BrickScreen.show("dis: " + dis);
			
			if (dis > disToGround && dis < 2.5f) {
				leftTargetSpeed = -1.2f * Tp2;
				rightTargetSpeed = Tp2;
			} else {
				leftTargetSpeed = 1.3f * Tp1;
				rightTargetSpeed = Tp1;
			}
			
			this.robot.getDrive().adjustRobotMovement(leftTargetSpeed, rightTargetSpeed);
			
		}
		
		if (flag) {
			this.robot.getMediumMotor().rotate(-20);
			this.robot.getDrive().turnLeft(8);
			this.robot.getDrive().travel(15);
		} else {
			this.robot.getMediumMotor().rotate(75);
		}
		this.robot.getDrive().stopWithMotors();
	}

}
