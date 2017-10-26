package mission;

import lejos.utility.TextMenu;

public class MissionMenu extends TextMenu {

	private static final String MENU_TITLE = "Parcours";
	private static final String[] missions = { Mission.LINE_FOLLOWING.getMission(), Mission.LABYRINTH.getMission(),
			Mission.OBSTACLE_SHIFTING.getMission(), Mission.BRIDGE.getMission() };

	public MissionMenu() {
		super(missions, 0, MENU_TITLE);
	}
	
	

}
