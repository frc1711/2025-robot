package frc.robot.configuration;

import edu.wpi.first.wpilibj.DriverStation;

public enum FieldThird {
	
	LEFT(20, 19, 13, 11, 6, 1),
	
	CENTER(21, 18, 13, 10, 7, 1),
	
	RIGHT(22, 17, 12, 9, 8, 2);
	
	public final int blueReefFrontTagID;
	
	public final int blueReefBackTagID;
	
	public final int blueCoralStationTagID;
	
	public final int redReefFrontTagID;
	
	public final int redReefBackTagID;
	
	public final int redCoralStationTagID;
	
	FieldThird(
		int blueReefFrontTagID,
		int blueReefBackTagID,
		int blueCoralStationTagID,
		int redReefFrontTagID,
		int redReefBackTagID,
		int redCoralStationTagID
	) {
		
		this.blueReefFrontTagID = blueReefFrontTagID;
		this.blueReefBackTagID = blueReefBackTagID;
		this.blueCoralStationTagID = blueCoralStationTagID;
		this.redReefFrontTagID = redReefFrontTagID;
		this.redReefBackTagID = redReefBackTagID;
		this.redCoralStationTagID = redCoralStationTagID;
		
	}
	
	public int getReefFrontAprilTagID(DriverStation.Alliance alliance) {
		
		return alliance == DriverStation.Alliance.Red
			? this.redReefFrontTagID
			: this.blueReefFrontTagID;
		
	}
	
	public int getReefFrontAprilTagID() {
		
		return this.getReefFrontAprilTagID(
			DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)
		);
		
	}
	
	public int getReefRearAprilTagID(DriverStation.Alliance alliance) {
		
		return alliance == DriverStation.Alliance.Red
			? this.redReefBackTagID
			: this.blueReefBackTagID;
		
	}
	
	public int getReefRearAprilTagID() {
		
		return this.getReefRearAprilTagID(
			DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)
		);
		
	}
	
	public int getCoralStationAprilTagID(DriverStation.Alliance alliance) {
		
		return alliance == DriverStation.Alliance.Red
			? this.redCoralStationTagID
			: this.blueCoralStationTagID;
		
	}
	
	public int getCoralStationAprilTagID() {
		
		return this.getCoralStationAprilTagID(
			DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)
		);
		
	}
	
	
	
}
