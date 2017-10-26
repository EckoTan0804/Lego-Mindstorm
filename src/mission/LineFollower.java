package mission;

import ev3Robot.Robot;

public class LineFollower implements Runnable{
	
	private static final float THRESHOLD = 0.5f; // TODO: need to be measured and adjusted
	
	private Robot robot;

	public LineFollower(Robot robot) {
		super();
		this.robot = robot;
	}

	@Override
	public void run() {
		this.lineFollowing();		
	}
	
	private void lineFollowing() {
		while(true) {
			if (this.robot.getSensorManager().getReflectedLightIntensity() < THRESHOLD) {
				// turn left
				this.robot.travelArc(1, 2);
			} else {
				// turn right
				this.robot.travelArc(-1, 2);
			}
		}
	}

}
