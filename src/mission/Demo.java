package mission;

import ev3Robot.Robot;
import lejos.hardware.Button;
import util.BrickScreen;

public class Demo {

	public static void main(String[] args) throws InterruptedException {
		Robot robot = new Robot();
		robot.mainLoop();
//		while (Button.ESCAPE.isUp()) {
//			robot.getDrive().setMotorSpeed(200, 200);
//			robot.getDrive().goForwardWithMotors();
//		}
//		robot.getDrive().quickStop();
//		BrickScreen.show("Stop!"); 
//		System.exit(0);
	}

}
