package mission;

import ev3Robot.Robot;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import util.BrickScreen;

public class Bridge {
	
	private final int RED = 0;
	private final int GREEN = 1;
	private final int BLUE = 2;
	private final float Tp = 300f;
	private final float disToGround = 0.07f;
	
	
	private Robot robot;
	
	

	public Bridge(Robot robot) {
		super();
		this.robot = robot;
	}

	public void startBridge() {

		BrickScreen.clearScreen();

		this.robot.changeSettingsForLabyrinth();
		
		this.robot.getMediumMotor().rotate(-70);

		try {
			Thread.sleep(20); 
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		float speed = Tp; // TODO: need to adjust the speed
		this.robot.getDrive().setMotorSpeed(speed, speed);
		this.robot.getDrive().goForwardWithMotors();
		float leftTargetSpeed = 0f;
		float rightTargetSpeed = 0f;
		
		while (Button.LEFT.isUp()) {
			
			BrickScreen.clearScreen();
			
			float[] rgb = this.robot.getSensors().getColorArray();
			float red = rgb[RED] * 255;
			float green = rgb[GREEN] * 255;
			float blue = rgb[BLUE] * 255;
			
			if (red <= 15 && blue > 16 && green <= 23) {
				break;
			}	
			
			float dis = this.robot.getSensors().getDistance();
			BrickScreen.show("dis: " + dis);
			
			if (dis > disToGround) {
				leftTargetSpeed = -1.2f * Tp;
				rightTargetSpeed = Tp;
			} else {
				leftTargetSpeed = 1.5f * Tp;
				rightTargetSpeed = Tp;
			}
			
			this.robot.getDrive().adjustRobotMovement(leftTargetSpeed, rightTargetSpeed);
			
		}

		this.robot.getMediumMotor().rotate(70);
		this.robot.getDrive().stopWithMotors();
	}

}
