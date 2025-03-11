package frc.robot.configuration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;

public enum ReefScoringMode {
	
	MANUAL("Manual"),
	
	LEFT("Left (auto align)"),
	
	RIGHT("Right (auto align)");
	
	public static final ReefScoringMode DEFAULT = LEFT;
	
	public static ReefScoringMode ACTIVE = DEFAULT;
	
	private static final String SENDABLE_NAME = "Reef Scoring Mode";
	
	protected final String humanReadableName;
	
	ReefScoringMode(String humanReadableName) {
		
		this.humanReadableName = humanReadableName;
		
	}
	
	public static void setMode(ReefScoringMode mode) {
		
		ReefScoringMode.ACTIVE = mode;
		
		SmartDashboard.putString(
			ReefScoringMode.SENDABLE_NAME,
			mode.getHumanReadableName()
		);
		
	}
	
	public static void nextMode() {
		
		ReefScoringMode.setMode(ReefScoringMode.ACTIVE.getNext());
		
	}
	
	public static void previousMode() {
		
		ReefScoringMode.setMode(ReefScoringMode.ACTIVE.getPrevious());
		
	}
	
	public String getHumanReadableName() {
		
		return this.humanReadableName;
		
	}
	
	public ReefScoringMode getNext() {
		
		List<ReefScoringMode> modes = List.of(ReefScoringMode.values());
		int currentIndex = modes.indexOf(this);
		int nextIndex = currentIndex + 1;
		
		if (nextIndex >= modes.size()) nextIndex = 0;
		
		return modes.get(nextIndex);
		
	}
	
	public ReefScoringMode getPrevious() {
		
		List<ReefScoringMode> modes = List.of(ReefScoringMode.values());
		int currentIndex = modes.indexOf(this);
		int previousIndex = currentIndex - 1;
		
		if (previousIndex < 0) previousIndex = modes.size() - 1;
		
		return modes.get(previousIndex);
		
	}
	
}
