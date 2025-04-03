package frc.robot.util;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.configuration.RobotDimensions;

import static edu.wpi.first.units.Units.*;;

public class ElevatorPosition {
	
	protected static final Distance MINIMUM_STAGE_1_TRAVEL_OFFSET =
		Inches.of(0);
	
	protected static final Distance MAXIMUM_STAGE_1_TRAVEL_OFFSET =
		Inches.of(25);
	
	public static final ElevatorPosition MINIMUM =
		ElevatorPosition.fromStage1TravelOffset(MINIMUM_STAGE_1_TRAVEL_OFFSET);
	
	public static final ElevatorPosition MAXIMUM =
		ElevatorPosition.fromStage1TravelOffset(MAXIMUM_STAGE_1_TRAVEL_OFFSET);
	
	public static final ElevatorPosition RESTING = MINIMUM;
//		ElevatorPosition.fromStage1TravelOffset(
//			MINIMUM_STAGE_1_TRAVEL_OFFSET.minus(Inches.of(0.05))
//		);
	
	public static final ElevatorPosition L2_ALGAE_REMOVAL =
		ElevatorPosition.fromUpperMailboxHeight(Inches.of(39));
	
	public static final ElevatorPosition L3_ALGAE_REMOVAL =
		ElevatorPosition.fromUpperMailboxHeight(Inches.of(55));
	
	public static final ElevatorPosition L1_SCORING = RESTING;
	
	public static final ElevatorPosition L1_LOADING =
		ElevatorPosition.fromLowerMailboxTravelOffset(
			RobotDimensions.UPPER_MAILBOX_RESTING_HEIGHT
				.minus(RobotDimensions.LOWER_MAILBOX_RESTING_HEIGHT)
				.minus(Inches.of(1))
		);
	
	public static final ElevatorPosition L2_SCORING = RESTING;
	
	public static final ElevatorPosition L3_SCORING =
//		ElevatorPosition.fromUpperMailboxHeight(Inches.of(50));
		ElevatorPosition.fromUpperMailboxHeight(Inches.of(49));
	
	public static final ElevatorPosition L4_SCORING =
//		ElevatorPosition.fromUpperMailboxHeight(Inches.of(75));
		ElevatorPosition.fromUpperMailboxHeight(Inches.of(74));
	
	public static final ElevatorPosition L4_DISTANT_SCORING =
		ElevatorPosition.fromUpperMailboxHeight(Inches.of(80));
	
	public static final ElevatorPosition L4_TIP =
//		ElevatorPosition.fromUpperMailboxHeight(Inches.of(83));
		ElevatorPosition.fromUpperMailboxHeight(Inches.of(82));
	
	protected final Distance stage1Offset;
	
	protected ElevatorPosition(Distance stage1Offset) {
		
		this.stage1Offset = stage1Offset;
		
	}
	
	public static ElevatorPosition fromMotorShaftAngle(Angle angle) {
		
		return ElevatorPosition.fromStage1TravelOffset(
			RobotDimensions.ELEVATOR_STAGE_1_TRAVEL_PER_MOTOR_REVOLUTION
				.times(angle.in(Rotations))
		);
		
	}
	
	public static ElevatorPosition fromElevatorShaftAngle(Angle angle) {
		
		return ElevatorPosition.fromStage1TravelOffset(
			RobotDimensions.ELEVATOR_STAGE_1_TRAVEL_PER_SHAFT_REVOLUTION
				.times(angle.in(Rotations))
		);
		
	}
	
	public static ElevatorPosition fromStage1TravelOffset(Distance offset) {
		
		return new ElevatorPosition(offset);
		
	}
	
	public static ElevatorPosition fromStage2TravelOffset(Distance offset) {
		
		return ElevatorPosition.fromStage1TravelOffset(offset.div(2));
		
	}
	
	public static ElevatorPosition fromUpperMailboxTravelOffset(Distance offset) {
		
		return ElevatorPosition.fromStage2TravelOffset(offset);
		
	}
	
	public static ElevatorPosition fromUpperMailboxHeight(Distance height) {
		
		return ElevatorPosition.fromStage2TravelOffset(
			height.minus(RobotDimensions.UPPER_MAILBOX_RESTING_HEIGHT)
		);
		
	}
	
	public static ElevatorPosition fromLowerMailboxTravelOffset(Distance offset) {
		
		return ElevatorPosition.fromStage2TravelOffset(offset);
		
	}
	
	public static ElevatorPosition fromLowerMailboxHeight(Distance height) {
		
		return ElevatorPosition.fromStage2TravelOffset(
			height.minus(RobotDimensions.LOWER_MAILBOX_RESTING_HEIGHT)
		);
		
	}
	
	public Angle getMotorShaftAngle() {
		
		return this.getOutputShaftAngle()
			.times(RobotDimensions.ELEVATOR_GEARBOX_RATIO);
		
	}
	
	public Angle getOutputShaftAngle() {
		
		return Rotations.of(
			this.stage1Offset.in(Inches) /
			RobotDimensions.ELEVATOR_STAGE_1_TRAVEL_PER_SHAFT_REVOLUTION.in(Inches)
		);
	
	}
	
	public Distance getStage1TravelOffset() {
		
		return this.stage1Offset;
		
	}
	
	public Distance getStage2TravelOffset() {
		
		return this.stage1Offset.times(2);
		
	}
	
	public Distance getUpperMailboxTravelOffset() {
		
		return this.getStage2TravelOffset();
		
	}
	
	public Distance getUpperMailboxHeight() {
		
		return this.getStage2TravelOffset()
			.plus(RobotDimensions.UPPER_MAILBOX_RESTING_HEIGHT);
		
	}
	
	public Distance getLowerMailboxTravelOffset() {
		
		return this.getStage2TravelOffset();
		
	}
	
	public Distance getLowerMailboxHeight() {
		
		return this.getStage2TravelOffset()
			.plus(RobotDimensions.LOWER_MAILBOX_RESTING_HEIGHT);
		
	}
	
}
