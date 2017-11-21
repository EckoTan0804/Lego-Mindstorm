package mission;

import ev3Robot.Robot;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.utility.TextMenu;

public class MissionMenu extends TextMenu {

	private static final String MENU_TITLE = "Parcours";
	// private static final String[] missions = {
	// Mission.LINE_FOLLOWING.getMission(), Mission.LABYRINTH.getMission(),
	// Mission.OBSTACLE_SHIFTING.getMission(), Mission.BRIDGE.getMission() };
	private static final String[] missions = Mission.getMissionStringList();

	public static final int MENU_ITEM_LINE_FOLLOWING = 0;
	public static final int MENU_ITEM_LABYRINTH = 1;
	public static final int MENU_ITEM_OBSTACLE_SHIFTING = 2;
	public static final int MENU_ITEM_BRIDGE = 3;

	public MissionMenu() {
		// super(missions, 0, MENU_TITLE);
		super(missions);
	}

	public void startGUI(Robot robot) throws InterruptedException {
		
		Sound.beepSequenceUp();
		
		while (Button.ESCAPE.isUp()) { // manual stop the program when Button "Escape" is pressed

			LCD.clear();

			// choose mission-routine from mission menu
			switch (this.select()) {

			case MissionMenu.MENU_ITEM_LINE_FOLLOWING:
				// show "Line following" on the display
				LCD.clear();
				LCD.drawString(Mission.LINE_FOLLOWING.getMission(), 0, 0);

				// line following routine will be executed
				LineFollower lineFollower = new LineFollower(robot);
				lineFollower.startLineFollowing();
				break;

			case MissionMenu.MENU_ITEM_LABYRINTH:
				// show "Labyrinth" on the display
				LCD.clear();
				LCD.drawString(Mission.LABYRINTH.getMission(), 0, 0);

				// labyrinth routine will be executed
				break;

			case MissionMenu.MENU_ITEM_OBSTACLE_SHIFTING:
				// show "Obstacle shifting" on the display
				LCD.clear();
				LCD.drawString(Mission.OBSTACLE_SHIFTING.getMission(), 0, 0);
				// obstacle shifting routine will be executed
				break;

			default:
				// show "Bridge" on the display
				LCD.clear();
				LCD.drawString(Mission.BRIDGE.getMission(), 0, 0);
				// bridge routine will be executed
				break;
			}

			Thread.sleep(20);
		}
	}

}
