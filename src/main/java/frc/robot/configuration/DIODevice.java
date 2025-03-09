package frc.robot.configuration;

public enum DIODevice {
	
	ELEVATOR_RELATIVE_ENCODER_CHANNEL_A(0),
	
	ELEVATOR_RELATIVE_ENCODER_CHANNEL_B(1),
	
	CLIMBER_ENCODER(2);
	
	public final int channel;
	
	DIODevice(int channel) {
		
		this.channel = channel;
		
	}
	
}
