package mission;

import ev3Robot.Robot;
import lejos.hardware.Button;

public class Demo {

	public static void main(String[] args) throws InterruptedException {
		Robot robot = new Robot();
		robot.mainLoop();
//		while (Button.ESCAPE.isUp()) {
//			robot.getDrive().setMotorSpeed(-100, 100);
//			robot.getDrive().goForwardWithMotors();
//		}
//		robot.getDrive().stopWithMotors();
	}

}
