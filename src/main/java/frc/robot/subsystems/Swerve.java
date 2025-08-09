// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.configuration.*;
import frc.robot.devicewrappers.RaptorsNavX;
import frc.robot.math.LinearMotionProfiler;
import frc.robot.util.*;

import java.util.function.IntSupplier;
import java.util.function.Supplier;
import java.util.stream.Stream;

import static edu.wpi.first.units.Units.*;

public class Swerve extends SubsystemBase {
	
	public static final LinearAcceleration MAX_LINEAR_ACCELERATION = FeetPerSecondPerSecond.of(12);

	public static final LinearAcceleration MAX_LINEAR_DECELERATION = FeetPerSecondPerSecond.of(-16);

	public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = RotationsPerSecondPerSecond.of(1);

	public static final AngularAcceleration MAX_ANGULAR_DECELERATION = RotationsPerSecondPerSecond.of(-2);

	public static final LinearVelocity MAX_LINEAR_VELOCITY = InchesPerSecond.of(100);

	public static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(0.5);

	public static final LinearVelocity SLOW_MODE_MAX_LINEAR_VELOCITY = InchesPerSecond.of(30);

	public static final AngularVelocity SLOW_MODE_MAX_ANGULAR_VELOCITY = DegreesPerSecond.of(60);

	protected final SwerveModule[] modules;
	
	protected final PIDController headingPIDController;
	
	protected final RaptorsNavX gyro;
	
	protected final SwerveDriveKinematics kinematics;
	
	protected final RaptorsOdometry odometry;
	
	public final Swerve.Commands commands;
	
	protected boolean isHeadingLockEnabled;
	
	public boolean isSlowModeEnabled;
	
	protected ChassisSpeeds chassisSpeeds;
	
	protected boolean shouldUseChassisSpeeds;
	
	public Swerve(RaptorsOdometry odometry) {
		
		this.modules = SwerveModuleConfiguration.getModuleConfigurations()
			.map(SwerveModule::new)
			.toArray(SwerveModule[]::new);
		
		this.headingPIDController = new PIDController(0, 0, 0);
		this.gyro = new RaptorsNavX();
		this.kinematics = new SwerveDriveKinematics(
			SwerveModuleConfiguration.getModuleConfigurations()
				.map(config -> config.positionInRobot)
				.toArray(Translation2d[]::new)
		);
		this.isHeadingLockEnabled = false;
		this.commands = new Swerve.Commands();
		this.odometry = odometry;
		this.isSlowModeEnabled = false;
		this.chassisSpeeds = new ChassisSpeeds(0, 0, 0);
		this.shouldUseChassisSpeeds = true;
		
		this.headingPIDController.enableContinuousInput(0, 360);
		
		this.calibrateFieldRelativeHeading(Degrees.of(180));
		
		ShuffleboardTab shuffleboardCalibrationTab =
			Shuffleboard.getTab("Calibration");
		
		shuffleboardCalibrationTab.add(
			this.commands.calibrateModuleSteeringHeadings()
		);
		
		shuffleboardCalibrationTab.add(
			this.commands.calibrateFieldRelativeHeading()
		);
		
		SmartDashboard.putData("Swerve Drive", this.getSwerveStateSendable());
		
	}
	
	public Stream<SwerveModule> getModuleStream() {
		
		return Stream.of(this.modules);
		
	}
	
	public SwerveModulePosition[] getModulePositions() {
		
		return this.getModuleStream()
			.map(SwerveModule::getPosition)
			.toArray(SwerveModulePosition[]::new);
		
	}
	
	protected void applyModuleStates(SwerveModuleState[] moduleStates) {
		
		this.getModuleStream().forEach(module -> {
			module.updateModuleState(moduleStates[module.getID()]);
		});
		
	}
	
	public SwerveDriveKinematics getKinematics() {
		
		return this.kinematics;
		
	}
	
	public void stop() {
		
		this.applyChassisSpeeds(new ChassisSpeeds(0, 0, 0));
		
	}
	
	public void setDriveMotorIdleState(SparkBaseConfig.IdleMode idleMode) {
		
		this.getModuleStream().forEach(module -> {
			
			SparkMaxConfig config = SwerveModule.getDriveMotorControllerConfig();
			
			config.idleMode(idleMode);
			
			module.driveMotorController.configure(
				config,
				SparkBase.ResetMode.kResetSafeParameters,
				SparkBase.PersistMode.kNoPersistParameters
			);
			
		});
		
	}
	
	public void calibrateFieldRelativeHeading() {
		
		this.calibrateFieldRelativeHeading(Degrees.of(0));
		
	}
	
	public void calibrateFieldRelativeHeading(Angle currentHeading) {
		
		this.gyro.yaw.calibrate(currentHeading);
		this.setFieldRelativeHeadingSetpoint(currentHeading.times(-1));
		
	}
	
	public Angle getFieldRelativeHeading() {
		
		return this.gyro.yaw.getAngle();
		
	}
	
	public ChassisSpeeds getActualChassisSpeeds() {

		return this.kinematics.toChassisSpeeds(
			this.getModuleStream()
				.map(SwerveModule::getActualState)
				.toArray(SwerveModuleState[]::new)
		);
		
	}
	
	public LinearVelocity getLinearVelocity() {
		
		ChassisSpeeds actualChassisSpeeds = this.getActualChassisSpeeds();
		Translation2d speedsTranslation = new Translation2d(
			actualChassisSpeeds.vxMetersPerSecond,
			actualChassisSpeeds.vyMetersPerSecond
		);
		
		return MetersPerSecond.of(speedsTranslation.getNorm());
		
	}
	
	public AngularVelocity getAngularVelocity() {
		
		return this.gyro.getYawAngularVelocity();
		
	}
	
	public void applyChassisSpeeds(ChassisSpeeds chassisSpeeds) {

		this.chassisSpeeds = chassisSpeeds;
		
	}
	
	public void setFieldRelativeHeadingSetpoint(Angle heading) {
		
		Pose2d existingPose = this.odometry.getPose();
		
		this.headingPIDController.setSetpoint(heading.in(Degrees));
		
		this.odometry.resetPose(existingPose);

	}
	
	@Override
	public void periodic() {

		if (!this.shouldUseChassisSpeeds) return;
		
		SwerveModuleState[] newModuleStates =
			this.kinematics.toSwerveModuleStates(this.chassisSpeeds);

		this.applyModuleStates(newModuleStates);

	}
	
	@Override
	public void initSendable(SendableBuilder builder) {
		
		builder.addDoubleProperty(
			"Heading",
			() -> this.getFieldRelativeHeading().in(Degrees),
			(double headingDegrees) -> this.setFieldRelativeHeadingSetpoint(Degrees.of(headingDegrees))
		);
		
		builder.addDoubleProperty(
			"Heading Setpoint",
			this.headingPIDController::getSetpoint,
			(double headingDegrees) -> this.setFieldRelativeHeadingSetpoint(Degrees.of(headingDegrees))
		);
		
		builder.addDoubleProperty(
			"Swerve Linear Velocity (in/sec)",
				() -> this.getLinearVelocity().in(InchesPerSecond),
				null
		);
		
		builder.addDoubleProperty(
			"Swerve Module Velocity kP",
			this.modules[0].driveMotorController.configAccessor.closedLoop::getP,
			(double kP) -> {
				SparkMaxConfig newConfig = new SparkMaxConfig();
				newConfig.closedLoop.p(kP);
				this.getModuleStream().forEach(module ->
					module.driveMotorController.configure(
						newConfig,
						SparkBase.ResetMode.kNoResetSafeParameters,
						SparkBase.PersistMode.kPersistParameters
					)
				);
			}
		);
		
		builder.addDoubleProperty(
			"Swerve Module Velocity kD",
			this.modules[0].driveMotorController.configAccessor.closedLoop::getD,
			(double kD) -> {
				SparkMaxConfig newConfig = new SparkMaxConfig();
				newConfig.closedLoop.d(kD);
				this.getModuleStream().forEach(module ->
					module.driveMotorController.configure(
						newConfig,
						SparkBase.ResetMode.kNoResetSafeParameters,
						SparkBase.PersistMode.kPersistParameters
					)
				);
			}
		);
		
		AngularVelocity vortexFreeSpeed = Rotations.per(Minute).of(6784);
		double driveWheelFreeSpeedRPS = (
			vortexFreeSpeed.in(RotationsPerSecond) *
				RobotDimensions.SWERVE_WHEEL_CIRCUMFERENCE.in(Inches)
		) / RobotDimensions.SWERVE_DRIVE_MOTOR_REDUCTION;
		double velocityFeedforward = 1 / driveWheelFreeSpeedRPS;
		
		builder.addDoubleProperty(
			"Swerve Module Velocity FF Add",
			() -> this.modules[0].driveMotorController.configAccessor.closedLoop.getFF() - velocityFeedforward,
			(double kFF) -> {
				SparkMaxConfig newConfig = new SparkMaxConfig();
				newConfig.closedLoop.velocityFF(velocityFeedforward + kFF);
				this.getModuleStream().forEach(module ->
					module.driveMotorController.configure(
						newConfig,
						SparkBase.ResetMode.kNoResetSafeParameters,
						SparkBase.PersistMode.kPersistParameters
					)
				);
			}
		);

		builder.addDoubleProperty(
			"Distance to Scoring Pose (in)",
				() -> {

					Pose2d currentPose = this.odometry.getPose();
					IntSupplier tagID = () -> this.odometry.getNearestReefAprilTag().ID;
					Pose2d leftScoringPose = RobotPoseBuilder.getReefScoringPose(tagID, ReefAlignment.LEFT).get();
					Pose2d rightScoringPose = RobotPoseBuilder.getReefScoringPose(tagID, ReefAlignment.RIGHT).get();
					double distanceToLeft = currentPose.minus(leftScoringPose).getTranslation().getNorm();
					double distanceToRight = currentPose.minus(rightScoringPose).getTranslation().getNorm();

					return distanceToLeft < distanceToRight
						? Meters.of(distanceToLeft).in(Inches)
						: Meters.of(distanceToRight).in(Inches);

				},
				null
		);
		
	}
	
	public Sendable getSwerveStateSendable() {
		
		return builder -> {
			
			builder.setSmartDashboardType("SwerveDrive");
			
			this.modules[0].addSendableFields(builder, "Front Left");
			this.modules[1].addSendableFields(builder, "Front Right");
			this.modules[2].addSendableFields(builder, "Back Left");
			this.modules[3].addSendableFields(builder, "Back Right");
			
			builder.addDoubleProperty(
				"Robot Angle",
				() -> this.getFieldRelativeHeading().in(Degrees),
				null
			);
			
			builder.addDoubleProperty(
				"Chassis Speeds (vX in inches per second)",
				() -> MetersPerSecond.of(this.getActualChassisSpeeds().vxMetersPerSecond).in(InchesPerSecond),
				null
			);
			
			builder.addDoubleProperty(
				"Chassis Speeds (vY in inches per second)",
				() -> MetersPerSecond.of(this.getActualChassisSpeeds().vyMetersPerSecond).in(InchesPerSecond),
				null
			);
			
			builder.addDoubleProperty(
				"Chassis Speeds (vXY in inches per second)",
				() -> this.getLinearVelocity().in(InchesPerSecond),
				null
			);
			
			builder.addDoubleProperty(
				"Chassis Speeds (Rotation in degrees per second)",
				() -> RadiansPerSecond.of(this.getActualChassisSpeeds().omegaRadiansPerSecond).in(DegreesPerSecond),
				null
			);
			
			builder.addStringProperty(
				"Field Position",
				() -> this.odometry.getFieldThird().name(),
				null
			);
			
			
			
		};
		
	}
	
	public Sendable getFieldPoseSendable() {
		
		return builder -> {
			
			double robotWidth = RobotDimensions.ROBOT_WIDTH.in(Meters);
			double robotLength = RobotDimensions.ROBOT_LENGTH.in(Meters);
			boolean showOtherObjects = true;
			boolean showTrajectories = true;
			double fieldRotation = Degrees.of(0).in(Degrees);
			int robotColor = Color.FRC_BLUE.toARGBInt();
			int trajectoryColor = Color.WHITE.toARGBInt();
			
			builder.addStringProperty("field_game", () -> "Reefscape", null);
			builder.addDoubleProperty("robot_width", () -> robotWidth, null);
			builder.addDoubleProperty("robot_length", () -> robotLength, null);
			builder.addBooleanProperty("show_other_objects", () -> showOtherObjects, null);
			builder.addBooleanProperty("show_trajectories", () -> showTrajectories, null);
			builder.addDoubleProperty("field_rotation", () -> fieldRotation, null);
			builder.addIntegerProperty("robot_color", () -> robotColor, null);
			builder.addIntegerProperty("trajectory_color", () -> trajectoryColor, null);
			
		};
		
	}
	
	public class Commands {
		
		public Command calibrateModuleSteeringHeadings() {

			return Swerve.this
				.runOnce(() -> Swerve.this.getModuleStream().forEach(SwerveModule::calibrateSteeringHeading))
				.withName("Calibrate Swerve Module Steering Headings")
				.ignoringDisable(true);

		}
		
		public Command calibrateFieldRelativeHeading() {
			
			return this.calibrateFieldRelativeHeading(Degrees.of(0));
			
		}
		
		public Command calibrateFieldRelativeHeading(Angle currentHeading) {
			
			return Swerve.this
				.runOnce(() -> Swerve.this.calibrateFieldRelativeHeading(currentHeading))
				.withName("Calibrate Swerve Field-relative Heading")
				.ignoringDisable(true);
			
		}
		
		public Command setFieldRelativeHeading(Angle heading) {
			
			return Swerve.this.runOnce(
				() -> Swerve.this.setFieldRelativeHeadingSetpoint(heading)
			);
			
		}
		
		public Command xMode(LinearVelocity outwardDriveSpeed) {
			
			return Swerve.this.startEnd(
				() -> {
					
					Swerve.this.shouldUseChassisSpeeds = false;
					
					Swerve.this.getModuleStream().forEach(module -> {
						module.updateModuleState(new SwerveModuleState(
							outwardDriveSpeed,
							module.config.positionInRobot.getAngle()
						));
					});
					
				},
				() -> {
					
					Swerve.this.shouldUseChassisSpeeds = true;
					Swerve.this.stop();
					
				}
			);
			
		}
		
		public Command xMode() {
			
			return this.xMode(InchesPerSecond.of(0));
			
		}
		
		public Command stop() {
			
			return new InstantCommand(Swerve.this::stop, Swerve.this);
			
		}

		public Command drive(Supplier<ChassisSpeeds> chassisSpeedsSupplier) {

			return Swerve.this.run(() -> Swerve.this.applyChassisSpeeds(chassisSpeedsSupplier.get()));

		}
		
		public Command goToPosition(
			Supplier<Pose2d> poseSupplier,
			Distance distanceTolerance,
			Angle angularTolerance,
			Supplier<int[]> aprilTagFilter
		) {
			
			AngularVelocity MAX_ANGULAR_VELOCITY = DegreesPerSecond.of(90);
			
			LinearMotionProfiler trajectory = new LinearMotionProfiler(
				/* Max Linear Velocity: */ InchesPerSecond.of(150),
				/* Max Linear Acceleration: */ FeetPerSecondPerSecond.of(40), 
				/* Max Linear Deceleration: */ FeetPerSecondPerSecond.of(6)
			);
			PIDController thetaController = new PIDController(8, 0, 0);
			thetaController.enableContinuousInput(-180, 180);

			Command command = new Command() {
				
				Pose2d desiredPose;
				Pose2d currentPose;
				
				@Override
				public void initialize() {
					
					if (aprilTagFilter != null) {
						Swerve.this.odometry.vision.setAprilTagFilter(aprilTagFilter.get());
					}

				}

				Distance getRemainingLinearDistance() {

					return Meters.of(
						currentPose.getTranslation()
							.minus(desiredPose.getTranslation())
							.getNorm()
					);

				}

				@Override
				public void execute() {
					
					Pose2d newCurrentPose = Swerve.this.odometry.getPose();
					Pose2d newDesiredPose = poseSupplier.get();
					
					if (newCurrentPose != null) this.currentPose = newCurrentPose;
					if (newDesiredPose != null) this.desiredPose = newDesiredPose;
					
					Swerve.this.odometry.setDisplaySetpoint(this.desiredPose);
					thetaController.setSetpoint(
						this.desiredPose.getRotation().getMeasure().in(Degrees)
					);

					LinearVelocity velocity = trajectory.calculate(
						this.getRemainingLinearDistance(),
						Swerve.this.getLinearVelocity()
					);
					
					Translation2d deltaTranslation = this.desiredPose.getTranslation()
						.minus(this.currentPose.getTranslation());
					
					Translation2d chassisSpeedTranslation = new Translation2d(
						velocity.in(MetersPerSecond),
						deltaTranslation.getAngle()
					).times(VirtualField.isRedAlliance() ? -1 : 1);
					
					Swerve.this.applyChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
						MetersPerSecond.of(chassisSpeedTranslation.getX()),
						MetersPerSecond.of(chassisSpeedTranslation.getY()),
						DegreesPerSecond.of(Math.min(
							thetaController.calculate(currentPose.getRotation().getDegrees()),
							MAX_ANGULAR_VELOCITY.in(DegreesPerSecond)
						))
					), Swerve.this.gyro.yaw.getRotation()));

				}

				@Override
				public boolean isFinished() {

					Pose2d relativePose = currentPose.relativeTo(desiredPose);
					Distance linearDistance = Meters.of(relativePose.getTranslation().getNorm());

					if (linearDistance.lt(Inches.of(0))) {
						
						linearDistance = linearDistance.times(-1);
						
					}

					return (
						linearDistance.lte(distanceTolerance) &&
						currentPose.getRotation().getMeasure().isNear(
							desiredPose.getRotation().getMeasure(),
							angularTolerance
						)
					);

				}

				@Override
				public void end(boolean interrupted) {

					Swerve.this.odometry.removeDisplaySetpoint();
					Swerve.this.odometry.vision.resetAprilTagFilter();
					Swerve.this.stop();

				}

			};

			command.addRequirements(Swerve.this);

			return command;

		}
		
		public Command waitUntilAtPosition(
			Supplier<Pose2d> desiredPoseSupplier,
			Distance distanceTolerance,
			Angle angularTolerance
		) {
			
			return edu.wpi.first.wpilibj2.command.Commands.waitUntil(() -> {
				
				Pose2d desiredPose = desiredPoseSupplier.get();
				Pose2d currentPose = Swerve.this.odometry.getPose();
				
				return (
					currentPose.getMeasureX().isNear(desiredPose.getMeasureX(), distanceTolerance) &&
					currentPose.getMeasureY().isNear(desiredPose.getMeasureY(), distanceTolerance) &&
					currentPose.getRotation().getMeasure().isNear(desiredPose.getRotation().getMeasure(), angularTolerance)
				);
				
			});
			
		}
	
	}
	
}
