package frc.robot.configuration;

import frc.robot.util.ElevatorPosition;

public enum ReefLevel {
	
	L1_TROUGH(ElevatorPosition.L1_SCORING),
	
	L2(ElevatorPosition.L2_SCORING),
	
	L3(ElevatorPosition.L3_SCORING),
	
	L4(ElevatorPosition.L4_SCORING);
	
	public final ElevatorPosition elevatorScoringPosition;

	ReefLevel(ElevatorPosition elevatorScoringPosition) {

		this.elevatorScoringPosition = elevatorScoringPosition;

	}
	
}
