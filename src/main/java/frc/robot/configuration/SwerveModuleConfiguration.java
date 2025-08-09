package frc.robot.configuration;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.stream.Stream;

public enum SwerveModuleConfiguration {
	
	FRONT_LEFT(
		0,
		CANDevice.SWERVE_FRONT_LEFT_STEER_MOTOR_CONTROLLER,
		CANDevice.SWERVE_FRONT_LEFT_DRIVE_MOTOR_CONTROLLER,
		DoublePreference.FRONT_LEFT_SWERVE_MODULE_ENCODER_OFFSET_DEGREES,
		true,
		true
	),
	
	FRONT_RIGHT(
		1,
		CANDevice.SWERVE_FRONT_RIGHT_STEER_MOTOR_CONTROLLER,
		CANDevice.SWERVE_FRONT_RIGHT_DRIVE_MOTOR_CONTROLLER,
		DoublePreference.FRONT_RIGHT_SWERVE_MODULE_ENCODER_OFFSET_DEGREES,
		true,
		false
	),
	
	REAR_LEFT(
		2,
		CANDevice.SWERVE_REAR_LEFT_STEER_MOTOR_CONTROLLER,
		CANDevice.SWERVE_REAR_LEFT_DRIVE_MOTOR_CONTROLLER,
		DoublePreference.REAR_LEFT_SWERVE_MODULE_ENCODER_OFFSET_DEGREES,
		false,
		true
	),
	
	REAR_RIGHT(
		3,
		CANDevice.SWERVE_REAR_RIGHT_STEER_MOTOR_CONTROLLER,
		CANDevice.SWERVE_REAR_RIGHT_DRIVE_MOTOR_CONTROLLER,
		DoublePreference.REAR_RIGHT_SWERVE_MODULE_ENCODER_OFFSET_DEGREES,
		false,
		false
	);
	
	public final int moduleID;
	
	public final CANDevice steerMotorControllerCANDevice;
	
	public final CANDevice driveMotorControllerCANDevice;
	
	public final DoublePreference steerEncoderOffset;
	
	public final boolean isFront;
	
	public final boolean isLeft;
	
	/**
	 * The position of this module within the robot.
	 */
	public final Translation2d positionInRobot;
	
	SwerveModuleConfiguration(
		int moduleID,
		CANDevice steerMotorControllerCANDevice,
		CANDevice driveMotorControllerCANDevice,
		DoublePreference steerEncoderOffset,
		boolean isFront,
		boolean isLeft
	) {
		
		this.moduleID = moduleID;
		this.steerMotorControllerCANDevice = steerMotorControllerCANDevice;
		this.driveMotorControllerCANDevice = driveMotorControllerCANDevice;
		this.steerEncoderOffset = steerEncoderOffset;
		this.isFront = isFront;
		this.isLeft = isLeft;
		this.positionInRobot = new Translation2d(
			RobotDimensions.SWERVE_FR_WHEELBASE_DISTANCE.div(2).times(isFront ? 1 : -1),
			RobotDimensions.SWERVE_LR_WHEELBASE_DISTANCE.div(2).times(isLeft ? 1 : -1)
		);
		
	}
	
	public static Stream<SwerveModuleConfiguration> getModuleConfigurations() {
		
		return Stream.of(
			FRONT_LEFT,
			FRONT_RIGHT,
			REAR_LEFT,
			REAR_RIGHT
		);
		
	}
	
}
