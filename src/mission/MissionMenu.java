package mission;

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
//		super(missions, 0, MENU_TITLE);
		super(missions);
	}

}
