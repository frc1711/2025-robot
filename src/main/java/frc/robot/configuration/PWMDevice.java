package frc.robot.configuration;

public enum PWMDevice {
	
	STATUS_LIGHTS_BLINKEN(0);
	
	public final int channel;
	
	PWMDevice(int channel) {
		
		this.channel = channel;
		
	}
	
}
