package frc.robot.configuration;

import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.*;

public final class RobotDimensions {
	
	public static final Distance PAN_LENGTH = Inches.of(29);
	
	public static final Distance PAN_WIDTH = Inches.of(29);
	
	public static final Distance BUMPER_THICKNESS = Inches.of(3.5);
	
	public static final Distance ROBOT_LENGTH =
		PAN_LENGTH.plus(BUMPER_THICKNESS.times(2));
	
	public static final Distance ROBOT_WIDTH =
		PAN_WIDTH.plus(BUMPER_THICKNESS.times(2));
	
	public static final Distance SWERVE_LR_WHEELBASE_DISTANCE = Inches.of(24.5);
	
	public static final Distance SWERVE_FR_WHEELBASE_DISTANCE = Inches.of(24.5);
	
	public static final Distance SWERVE_WHEEL_DIAMETER = Inches.of(3);
	
	public static final Distance SWERVE_WHEEL_CIRCUMFERENCE =
		SWERVE_WHEEL_DIAMETER.times(Math.PI);
	
	public static final double SWERVE_DRIVE_MOTOR_REDUCTION =
		(45.0 * 22.0) / (14.0 * 15.0);
	
	public static final Distance GROUND_CLEARANCE = Inches.of(2.4375);
	
	public static final Distance PAN_THICKNESS = Inches.of(0.125);
	
	public static final Distance ELEVATOR_STAGE_1_RESTING_HEIGHT_FROM_PAN =
		Inches.of(2.5);
	
	public static final Distance ELEVATOR_STAGE_2_RESTING_HEIGHT_FROM_PAN =
		ELEVATOR_STAGE_1_RESTING_HEIGHT_FROM_PAN.plus(Inches.of(1.875));
	
	public static final Distance LOWER_MAILBOX_RESTING_HEIGHT = Inches.of(23);
	
	public static final Distance UPPER_MAILBOX_RESTING_HEIGHT = Inches.of(33);
	
	public static final double ELEVATOR_GEARBOX_RATIO = 15;
	
	public static final Distance N25_CHAIN_PITCH = Inches.of(0.25);
	
	public static final double ELEVATOR_SPROCKET_TOOTH_COUNT = 14;
	
	public static final double REV_THROUGH_BORE_ENCODER_COUNTS_PER_REV = 2048;
	
	public static final Distance ELEVATOR_STAGE_1_TRAVEL_PER_SHAFT_REVOLUTION =
			N25_CHAIN_PITCH.times(ELEVATOR_SPROCKET_TOOTH_COUNT);
	
	public static final Distance ELEVATOR_STAGE_1_TRAVEL_PER_MOTOR_REVOLUTION =
			ELEVATOR_STAGE_1_TRAVEL_PER_SHAFT_REVOLUTION.div(ELEVATOR_GEARBOX_RATIO);
	
	public static final Distance MAILBOX_LR_OFFSET_TO_ROBOT_CENTER =
		Inches.of(11);
	
	public static final Distance REEF_BRANCH_SEPARATION_DISTANCE =
		Inches.of(13);
	
}
