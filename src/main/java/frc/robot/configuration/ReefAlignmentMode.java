package frc.robot.configuration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;

public enum ReefAlignmentMode {
	
	MANUAL("Manual"),
	
	LEFT("Left"),
	
	RIGHT("Right");
	
	public static final ReefAlignmentMode DEFAULT = LEFT;
	
	public static ReefAlignmentMode ACTIVE = DEFAULT;
	
	private static final String SENDABLE_NAME = "Reef Alignment Mode";
	
	protected final String humanReadableName;
	
	ReefAlignmentMode(String humanReadableName) {
		
		this.humanReadableName = humanReadableName;
		
	}
	
	public static void setMode(ReefAlignmentMode mode) {
		
		ReefAlignmentMode.ACTIVE = mode;
		
		SmartDashboard.putString(
			ReefAlignmentMode.SENDABLE_NAME,
			mode.getHumanReadableName()
		);
		
	}
	
	public static void nextMode() {
		
		ReefAlignmentMode.setMode(ReefAlignmentMode.ACTIVE.getNext());
		
	}
	
	public static void previousMode() {
		
		ReefAlignmentMode.setMode(ReefAlignmentMode.ACTIVE.getPrevious());
		
	}
	
	public String getHumanReadableName() {
		
		return this.humanReadableName;
		
	}
	
	public ReefAlignmentMode getNext() {
		
		List<ReefAlignmentMode> modes = List.of(ReefAlignmentMode.values());
		int currentIndex = modes.indexOf(this);
		int nextIndex = currentIndex + 1;
		
		if (nextIndex >= modes.size()) nextIndex = 0;
		
		return modes.get(nextIndex);
		
	}
	
	public ReefAlignmentMode getPrevious() {
		
		List<ReefAlignmentMode> modes = List.of(ReefAlignmentMode.values());
		int currentIndex = modes.indexOf(this);
		int previousIndex = currentIndex - 1;
		
		if (previousIndex < 0) previousIndex = modes.size() - 1;
		
		return modes.get(previousIndex);
		
	}
	
}
