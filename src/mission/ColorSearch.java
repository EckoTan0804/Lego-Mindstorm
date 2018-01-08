	package mission;

import ev3Robot.Robot;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import util.BrickScreen;

public class ColorSearch {
	
	private Robot robot;
	

	private final int RED = 0;
	private final int GREEN = 1;
	private final int BLUE = 2;
	private final float EPS = 20f;
	private final float disF = 5;
	
	private boolean found = false;
	private float r;
	private float g;
	private float b;
	private float backGroundR;
	private float backGroundG;
	private float backGroundB;
	private float turnL;
	
	
	
	public ColorSearch(Robot robot) {
		this.robot = robot;
	}
	
	public void startColorSearch() {


		BrickScreen.clearScreen();

		this.robot.changeSettingsForLabyrinth();

		try {
			Thread.sleep(20);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		float[] rgb = this.robot.getSensors().getColorArray();
		backGroundR = rgb[RED] * 255;
		backGroundG = rgb[GREEN] * 255;
		backGroundB = rgb[BLUE] * 255;
		this.robot.getDrive().rotate(100);
		this.robot.getDrive().setMotorSpeed(500, 500);
		this.robot.getDrive().goForwardWithMotors();
		turnL = -1;
		
		while(Button.LEFT.isUp()) {
			
			
			BrickScreen.clearScreen();
			
			rgb = this.robot.getSensors().getColorArray();
			float red = rgb[RED] * 255;
			float green = rgb[GREEN] * 255;
			float blue = rgb[BLUE] * 255;
			

			
			BrickScreen.show("r: " + red);
			BrickScreen.show("g: " + green);
			BrickScreen.show("b: " + blue);
			BrickScreen.show("found: " + found);
			
			float diff = Math.abs(red - backGroundR) + Math.abs(blue - backGroundB) 
						+ Math.abs(green - backGroundG);
			
			BrickScreen.show("diff: " + diff);
			
			if (diff > EPS) {
				if (!found) {
					this.robot.getDrive().stopWithMotors();
					Sound.setVolume(70);
					Sound.beepSequenceUp();;
					this.robot.getDrive().goForwardWithMotors();
					this.r = red;
					this.g = green;
					this.b = blue;
					found = true;
				} else {
					diff = Math.abs(red - r) + Math.abs(blue - b) 
							+ Math.abs(green - g);
					if (diff > EPS) {
						this.robot.getDrive().stopWithMotors();
						break;
					}
				}
			}
			
			
			if ((this.robot.getSensors().getTouch1()) > 0.2 && (this.robot.getSensors().getTouch2()) > 0.2) {
				/* special case: the robot reaches an obstacle */
				turn();
			}
			
		}

		this.robot.getDrive().stopWithMotors();
		
	}
	
	public void turn() {
			this.robot.getDrive().travel(-disF);
			this.robot.getDrive().rotate(turnL * 105);
			this.robot.getDrive().travel(1.5 * disF);
			this.robot.getDrive().rotate(turnL * 95);
			this.robot.getDrive().goForwardWithMotors();
			turnL = - turnL;
	}

}
