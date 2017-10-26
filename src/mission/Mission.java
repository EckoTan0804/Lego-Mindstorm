package mission;

public enum Mission {
	LINE_FOLLOWING("Line following"),
	LABYRINTH("Labyrinth"),
	OBSTACLE_SHIFTING("Obstacle shifting"),
	BRIDGE("Bridge");
	
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
	
	
}
