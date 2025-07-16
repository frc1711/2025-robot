// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.configuration.RobotDimensions;
import frc.robot.configuration.SwerveModuleConfiguration;
import frc.robot.math.DoubleUtilities;

import static edu.wpi.first.units.Units.*;

public class SwerveModule extends SubsystemBase {
	
	public final SwerveModuleConfiguration config;
	
	/**
	 * The motor controller that controls the steer motor of this module.
	 */
	protected final SparkMax steerMotorController;
	
	/**
	 * The motor controller that controls the drive motor of this module.
	 */
	public final SparkFlex driveMotorController;
	
	/**
	 * The absolute encoder that measures the current heading of this module.
	 */
	protected final SparkAbsoluteEncoder steerEncoder;
	
	/**
	 * The relative encoder that measures the current distance of this module.
	 */
	protected final RelativeEncoder driveEncoder;
	
	protected final SparkClosedLoopController steerController;
	
	protected final SparkClosedLoopController driveController;
	
	protected Angle steerAngleSetpoint;
	
	protected LinearVelocity driveVelocitySetpoint;
	
	/**
	 * Initializes a new SwerveModule with the given motor controllers and
	 * encoders.
	 *
	 * @param config The configuration for this swerve module.
	 */
	public SwerveModule(
		SwerveModuleConfiguration config
	) {
		
		this.config = config;
		
		this.steerMotorController = new SparkMax(
			config.steerMotorControllerCANDevice.id,
			SparkLowLevel.MotorType.kBrushless
		);
		
		this.driveMotorController = new SparkFlex(
			config.driveMotorControllerCANDevice.id,
			SparkLowLevel.MotorType.kBrushless
		);
		
		this.steerEncoder = this.steerMotorController.getAbsoluteEncoder();
		this.driveEncoder = this.driveMotorController.getEncoder();
		
		this.steerController = this.steerMotorController.getClosedLoopController();
		this.driveController = this.driveMotorController.getClosedLoopController();
		
		this.steerAngleSetpoint = Degrees.of(0);
		this.driveVelocitySetpoint = InchesPerSecond.of(0);
		
		this.driveEncoder.setPosition(0);
		this.config.steerEncoderOffset.useValue(degrees -> {
			
			SparkMaxConfig newConfig = new SparkMaxConfig();
			
			newConfig.absoluteEncoder.zeroOffset(
				DoubleUtilities.normalizeToRange(
					Degrees.of(degrees).in(Rotations),
					0,
					1
				)
			);
			
			this.steerMotorController.configure(
				newConfig,
				SparkBase.ResetMode.kNoResetSafeParameters,
				SparkBase.PersistMode.kNoPersistParameters
			);
			
		});
		
		this.steerMotorController.configure(
			SwerveModule.getSteerMotorControllerConfig(),
			SparkBase.ResetMode.kResetSafeParameters,
			SparkBase.PersistMode.kNoPersistParameters
		);
		
		this.driveMotorController.configure(
			SwerveModule.getDriveMotorControllerConfig(),
			SparkBase.ResetMode.kResetSafeParameters,
			SparkBase.PersistMode.kNoPersistParameters
		);
		
		this.steerController.setReference(
			0,
			SparkBase.ControlType.kPosition
		);
		
		// 0.005 volts/degree ~= 1.7 volts/rotation divided by 360 degrees/rotation
//		this.steerFeedforward = new SimpleMotorFeedforward(0.5, 0.005);
	
	}
	
	protected static SparkMaxConfig getSteerMotorControllerConfig() {
		
		SparkMaxConfig config = new SparkMaxConfig();
		
		config.idleMode(SparkBaseConfig.IdleMode.kBrake);
		config.smartCurrentLimit(20);
		
		config.absoluteEncoder
			// The output shaft of the rotates in the opposite direction of the
			// motor shaft for the steering motor.
			.inverted(true)
			// The default position units are rotations -- convert to degrees.
			.positionConversionFactor(360)
			// The default velocity units are rotations per minute -- convert to
			// degrees per second.
			.velocityConversionFactor(360.0 / 60.0)
			.zeroCentered(true);
		
		config.closedLoop
			.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
			.pid(0.01, 0, 0)
//			.velocityFF(1/0.47869)
			.outputRange(-1, 1)
			.positionWrappingEnabled(true)
			.positionWrappingInputRange(-180, 180);
		
//		config.closedLoop.maxMotion
			// Set a maximum angular error of 1 degree.
//			.allowedClosedLoopError(1)
			// Set a maximum angular velocity of 180 degrees per second.
//			.maxVelocity(180)
//			.maxVelocity(360)
			// Set a maximum angular acceleration such that we can reach maximum
			// velocity in 1/4 of a second.
//			.maxAcceleration(180.0 / 4);
//			.maxAcceleration(90)
//			.positionMode(MAXMotionConfig.MAXMotionPositionMode.kMAXMotionTrapezoidal);
		
		config.signals
			.absoluteEncoderPositionAlwaysOn(true)
			.absoluteEncoderVelocityAlwaysOn(true)
			.appliedOutputAlwaysOn(true);
			
		return config;
		
	}
	
	public static SparkMaxConfig getDriveMotorControllerConfig() {
		
		SparkMaxConfig config = new SparkMaxConfig();
		
		config.idleMode(SparkBaseConfig.IdleMode.kBrake);
		config.smartCurrentLimit(50);
		
		double conversionFactor =
			RobotDimensions.SWERVE_WHEEL_CIRCUMFERENCE.in(Inches) /
			RobotDimensions.SWERVE_DRIVE_MOTOR_REDUCTION;
		
		config.encoder
			.positionConversionFactor(conversionFactor)
			.velocityConversionFactor(conversionFactor / 60.0);
		
		AngularVelocity vortexFreeSpeed = Rotations.per(Minute).of(6784);
		double driveWheelFreeSpeedRPS = (
			vortexFreeSpeed.in(RotationsPerSecond) *
				RobotDimensions.SWERVE_WHEEL_CIRCUMFERENCE.in(Inches)
		) / RobotDimensions.SWERVE_DRIVE_MOTOR_REDUCTION;
		double velocityFeedforward = 1 / driveWheelFreeSpeedRPS;
		
		config.closedLoop
			.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
			.pid(0.01, 0, 0.1)
//			.velocityFF(velocityFeedforward + 0.002)
//			.velocityFF(velocityFeedforward + 0.003)
			.velocityFF(velocityFeedforward + 0.001)
			.outputRange(-1, 1)
			.maxMotion
				.maxVelocity(80)
				.maxAcceleration(160);
		
		return config;
		
	}
	
	public int getID() {
		
		return this.config.moduleID;
		
	}
	
	public SwerveModulePosition getPosition() {
		
		return new SwerveModulePosition(
			Inches.of(driveEncoder.getPosition()),
			new Rotation2d(this.getSteeringHeading())
		);
		
	}
	
	/**
	 * Returns the current heading of this swerve module, oriented in the
	 * standard FRC 'northwest-up' ('NWU') coordinate system.
	 *
	 * @return The current heading of this swerve module.
	 */
	public Angle getSteeringHeading() {
		
		// Normalize the steering angle value to the range [-180, 180].
		return Degrees.of(DoubleUtilities.normalizeToRange(
			this.steerEncoder.getPosition(),
			-180,
			180
		));
		
	}
	
	/**
	 * Calibrates the steering heading for this swerve module such that it will
	 * return 0 degrees in its current physical position after this method is
	 * called.
	 *
	 * @see #calibrateSteeringHeading(Angle)
	 */
	public void calibrateSteeringHeading() {
		
		this.calibrateSteeringHeading(Degrees.of(0));
		
	}
	
	/**
	 * Calibrates the steering heading for this swerve module such that it will
	 * return the given angle in its current physical position after this method
	 * is called.
	 *
	 * @param currentHeading The heading to calibrate this swerve module at.
	 */
	public void calibrateSteeringHeading(Angle currentHeading) {
		
		Angle existingOffset = Rotations.of(
			this.steerMotorController
				.configAccessor
				.absoluteEncoder
				.getZeroOffset()
		);
		
		this.config.steerEncoderOffset.set(
			existingOffset
				.minus(this.getSteeringHeading())
				.minus(currentHeading)
				.in(Degrees)
		);
		
	}
	
	public Angle getSteeringHeadingSetpoint() {
		
		return this.steerAngleSetpoint;
		
	}
	
	/**
	 * Returns a measurement of the error between the current steering heading
	 * and the setpoint of the steering PID controller.
	 *
	 * @return A measurement of the error between the current steering heading
	 * and the setpoint of the steering PID controller.
	 */
	public Angle getSteeringHeadingError() {
		
		Angle steerHeadingError = this.getSteeringHeading()
			.minus(this.getSteeringHeadingSetpoint());
		
		return Degrees.of(DoubleUtilities.normalizeToRange(
			steerHeadingError.in(Degrees),
			-180,
			180
		));
		
	}
	
	public LinearVelocity getVelocity() {
		
		return Inches.per(Second).of(this.driveEncoder.getVelocity());
		
	}
	
	public LinearVelocity getVelocitySetpoint() {
		
		return this.driveVelocitySetpoint;
		
	}
	
	public void updateModuleState(SwerveModuleState newState) {
		
		Rotation2d currentSteeringHeading =
			new Rotation2d(this.getSteeringHeading());

		newState.optimize(currentSteeringHeading);
		
		// Perform cosine speed compensation.
		newState.speedMetersPerSecond *=
			newState.angle.minus(currentSteeringHeading).getCos();
		
		this.steerAngleSetpoint = newState.angle.getMeasure();
		this.driveVelocitySetpoint =
			MetersPerSecond.of(newState.speedMetersPerSecond);

		this.steerController.setReference(
			this.steerAngleSetpoint.in(Degrees),
			SparkBase.ControlType.kPosition
		);
		
		this.driveController.setReference(
			this.driveVelocitySetpoint.in(InchesPerSecond),
			SparkBase.ControlType.kVelocity
		);
		
		
		
	}
	
	public SysIdRoutine getSteerMotorSysIdRoutine() {
		
		return new SysIdRoutine(
			new SysIdRoutine.Config(),
			new SysIdRoutine.Mechanism(
				this.steerMotorController::setVoltage,
				null,
				this
			)
		);
		
	}
	
	public void addSendableFields(SendableBuilder builder, String moduleName) {
		
		builder.addDoubleProperty(
			moduleName + " Angle",
			() -> this.getSteeringHeading().in(Degrees),
			null
		);
		
		builder.addDoubleProperty(
			moduleName + " Angle Setpoint",
			() -> this.getSteeringHeadingSetpoint().in(Degrees),
			null
		);
		
		builder.addDoubleProperty(
			moduleName + " Velocity",
			() -> this.getVelocity().in(MetersPerSecond),
			null
		);
		
		builder.addDoubleProperty(
			moduleName + " Velocity (inches per second)",
			() -> this.getVelocity().in(InchesPerSecond),
			null
		);
		
//		LinearFilter velocityFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
		LinearFilter velocityFilter = LinearFilter.movingAverage(5);
		
		builder.addDoubleProperty(
			moduleName + " Velocity Avg. (inches per second)",
			() -> velocityFilter.calculate(this.getVelocity().in(InchesPerSecond)),
			null
		);
		
		builder.addDoubleProperty(
			moduleName + " Velocity Setpoint (inches per second)",
			() -> this.getVelocitySetpoint().in(InchesPerSecond),
			null
		);
		
		builder.addDoubleProperty(
			moduleName + " Velocity Error (inches per second)",
			() -> this.getVelocity().minus(this.getVelocitySetpoint()).in(InchesPerSecond),
			null
		);
		
	}
	
	@Override
	public void initSendable(SendableBuilder builder) {
		
//		builder.addDoubleProperty(
//			"heading",
//			() -> this.getSteeringHeading().in(Degrees),
//			null
//		);
//
//		builder.addDoubleProperty(
//			"drive speed",
//			this.driveMotorController::get,
//			null
//		);
//
//		builder.addDoubleProperty(
//			"heading setpoint",
//			this.steerPIDController::getSetpoint,
//			this.steerPIDController::setSetpoint
//		);
//
//		builder.addDoubleProperty(
//			"heading error",
//			() -> this.getSteeringHeadingError().in(Degrees),
//			null
//		);
//
//		builder.addDoubleProperty(
//			"drive speed setpoint",
//			() -> this.drivePIDController.getSetpoint(),
//			null
//		);
//
//		builder.addDoubleProperty(
//			"Encoder Rotation (Degrees)",
//			() -> getEncoderRotation().getDegrees(),
//			null
//		);
//
//		builder.addDoubleProperty(
//			"Steer Speed",
//			() -> steerSpeed,
//			null
//		);
//
//		builder.addDoubleProperty(
//			"Drive Speed",
//			() -> driveSpeed,
//			null
//		);
//
//		builder.addDoubleProperty(
//			"distance-Readout",
//			() -> driveEncoder.getPosition() * .1 - distanceOffset,
//			null
//		);
//
//		builder.addDoubleProperty(
//			"distance-Offset",
//			() -> distanceOffset,
//			null
//		);
		
	}
	
}
