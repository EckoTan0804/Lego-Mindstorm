package sensor;

public class SensorThread implements Runnable {
	
	// "s" stands for "sensor"
	private SingleValueSensorWrapper sTouch1;
	private SingleValueSensorWrapper sTouch2;
	private SingleValueSensorWrapper sColor;
	private SingleValueSensorWrapper sDistance;
	
	private float touch1;
	private float touch2;
	private float color;
	private float distance;
	
	public SensorThread(SingleValueSensorWrapper sColor, SingleValueSensorWrapper sTouch1, SingleValueSensorWrapper sTouch2, SingleValueSensorWrapper sDistance) {
		this.sColor = sColor;
		this.sTouch1 = sTouch1;
		this.sTouch2 = sTouch2;
		this.sDistance = sDistance;
	}
	
	@Override
	public void run() {
		try {
			while(true) {
				this.color = this.sColor.getSample();
				this.touch1 = this.sTouch1.getSample();
				this.touch2 = this.sTouch2.getSample();
				this.distance = this.sDistance.getSample();
				Thread.sleep(20);
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

	public float getTouch1() {
		return touch1;
	}

	public void setTouch1(float touch1) {
		this.touch1 = touch1;
	}

	public float getTouch2() {
		return touch2;
	}

	public void setTouch2(float touch2) {
		this.touch2 = touch2;
	}

	public float getColor() {
		return color;
	}

	public void setColor(float color) {
		this.color = color;
	}

	public float getDistance() {
		return distance;
	}

	public void setDistance(float distance) {
		this.distance = distance;
	}
	
	

}
