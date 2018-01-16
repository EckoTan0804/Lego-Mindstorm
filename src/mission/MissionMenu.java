package mission;

import ev3Robot.Robot;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.utility.TextMenu;
import util.BrickScreen;

public class MissionMenu extends TextMenu {

	private static final String[] missions = Mission.getMissionStringList();

	public static final int MENU_ITEM_LINE_FOLLOWING = 0;
	public static final int MENU_ITEM_LABYRINTH = 1;
	public static final int MENU_ITEM_OBSTACLE_SHIFTING = 2;
	public static final int MENU_ITEM_BRIDGE = 3;
	public static final int MENU_ITEM_COLOR_SEARCH = 4;
	public static final int MENU_ITEM_PARKOUR = 5;

	public MissionMenu() {
		super(missions);
	}

	public void startGUI(Robot robot) throws InterruptedException {
		
		Sound.beepSequenceUp();
		
		LineFollower lineFollower = new LineFollower(robot);
		Labyrinth labyrinth = new Labyrinth(robot);
		ColorSearch colorSearch = new ColorSearch(robot);
		Bridge bridge = new Bridge(robot);
		
		while (Button.ESCAPE.isUp()) { /* manual stop the program when Button "Escape" is pressed */

			BrickScreen.clearScreen();

			/* choose mission-routine from mission menu */
			switch (this.select()) {

			case MissionMenu.MENU_ITEM_LINE_FOLLOWING:
				BrickScreen.clearScreen();

				/* line following routine will be executed. */
				
				lineFollower.startLineFollowing();
				break;

			case MissionMenu.MENU_ITEM_LABYRINTH:
				
				BrickScreen.clearScreen();

				/* labyrinth routine will be executed. */
				
				labyrinth.reset();
				labyrinth.startLabyrinth();
				break;

			case MissionMenu.MENU_ITEM_OBSTACLE_SHIFTING:

				BrickScreen.clearScreen();
				
				/* obstacle shifting routine will be executed. */
				break;
			
			case MissionMenu.MENU_ITEM_COLOR_SEARCH:
				
				BrickScreen.clearScreen();

				colorSearch.startColorSearch();
				break;
				
			case MissionMenu.MENU_ITEM_BRIDGE:

				BrickScreen.clearScreen();
				
				/* bridge routine will be executed. */
				
				bridge.startBridge();
				
				
				break;

			default:
				
				BrickScreen.clearScreen();
				
				/* The whole parkour will be executed. */
				
				lineFollower.startLineFollowing();
				lineFollower.fromLineFollowerToLabyrinth();
				labyrinth.startLabyrinth();
				labyrinth.fromLabyrinthToBridge();
				bridge.startBridge();
				colorSearch.startColorSearch();
			}

			Thread.sleep(20);
		}
	}

}
