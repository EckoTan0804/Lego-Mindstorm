package mission;

public enum Mission {
	LINE_FOLLOWING("Line following"), LABYRINTH("Labyrinth"), OBSTACLE_SHIFTING("Obstacle shifting"), BRIDGE("Bridge");

	private String mission;

	private Mission(String mission) {
		this.mission = mission;
	}

	public String getMission() {
		return mission;
	}

	public void setMission(String mission) {
		this.mission = mission;
	}

	public static String[] getMissionStringList() {
		String[] missonStringList = { LINE_FOLLOWING.getMission(), LABYRINTH.getMission(),
				OBSTACLE_SHIFTING.getMission(), BRIDGE.getMission() };
		return missonStringList;
	}

}
