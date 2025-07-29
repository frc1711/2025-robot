// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
import frc.robot.util.*;

import java.util.*;
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
		
		this.gyro.calibrate(currentHeading);
		this.setFieldRelativeHeadingSetpoint(currentHeading.times(-1));
		
	}
	
	public Angle getFieldRelativeHeading() {
		
		return this.gyro.getRotation();
		
	}
	
	public LinearVelocity getLinearVelocity() {
		
		return MetersPerSecond.of(new Translation2d(
			this.chassisSpeeds.vxMetersPerSecond,
			this.chassisSpeeds.vyMetersPerSecond
		).getNorm());
		
	}
	
	public AngularVelocity getAngularVelocity() {
		
		return this.gyro.getAngularVelocity();
		
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
					int tagID = this.odometry.getNearestReefAprilTag().ID;
					Pose2d leftScoringPose = RobotPoseBuilder.getReefScoringPose(tagID, ReefAlignment.LEFT).toPose();
					Pose2d rightScoringPose = RobotPoseBuilder.getReefScoringPose(tagID, ReefAlignment.RIGHT).toPose();
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
				() -> MetersPerSecond.of(this.chassisSpeeds.vxMetersPerSecond).in(InchesPerSecond),
				null
			);
			
			builder.addDoubleProperty(
				"Chassis Speeds (vY in inches per second)",
				() -> MetersPerSecond.of(this.chassisSpeeds.vyMetersPerSecond).in(InchesPerSecond),
				null
			);
			
			builder.addDoubleProperty(
				"Chassis Speeds (vXY in inches per second)",
				() -> this.getLinearVelocity().in(InchesPerSecond),
				null
			);
			
			builder.addDoubleProperty(
				"Chassis Speeds (Rotation in degrees per second)",
				() -> RadiansPerSecond.of(this.chassisSpeeds.omegaRadiansPerSecond).in(DegreesPerSecond),
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

	public enum DriveMode {
		ROBOT_RELATIVE,
		FIELD_RELATIVE,
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
			LinearVelocity maxLinearVelocity,
			Distance distanceTolerance,
			Angle angularTolerance,
			Supplier<int[]> aprilTagFilter
		) {
			
			Command command = new Command() {
				
//				final double LINEAR_KP = 0.000000005;
				final double LINEAR_KP = 0.000000001;
				
				final double ANGULAR_KP = 8;
				
				final AngularVelocity MAX_ANGULAR_VELOCITY = DegreesPerSecond.of(90);
				
				final PIDController xController =
					new PIDController(LINEAR_KP, 0, 0);
				
				final PIDController yController =
					new PIDController(LINEAR_KP, 0, 0);
				
				final PIDController thetaController =
					new PIDController(ANGULAR_KP, 0, 0);
				
				Pose2d desiredPose;
				Pose2d currentPose;
				
				@Override
				public void initialize() {
					
					this.thetaController.enableContinuousInput(-180, 180);
					this.updateSetpoint();
					if (aprilTagFilter != null) {
						Swerve.this.odometry.vision.setAprilTagFilter(aprilTagFilter.get());
					}
					Swerve.this.odometry.setDisplaySetpoint(desiredPose);
				
				}
				
				void updateSetpoint() {
					
					desiredPose = poseSupplier.get();
					
					xController.setSetpoint(desiredPose.getMeasureX().in(Inches));
					yController.setSetpoint(desiredPose.getMeasureY().in(Inches));
					thetaController.setSetpoint(desiredPose.getRotation().getMeasure().in(Degrees));
					
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
					
					if (newCurrentPose != null) this.currentPose = newCurrentPose;
					
					double inchesRemaining = this.getRemainingLinearDistance()
						.in(Inches);
					double degreesRemaining = currentPose.getRotation()
						.minus(desiredPose.getRotation())
						.getDegrees();
					
					System.out.printf(
						"goToPosition remaining travel: [%.2f inches, %.2f degrees]\n",
						inchesRemaining,
						degreesRemaining
					);
					
					double thetaFeed = thetaController.calculate(currentPose.getRotation().getDegrees());
					Translation2d point = new Translation2d(
						Inches.of(xController.calculate(currentPose.getMeasureX().in(Inches))),
						Inches.of(yController.calculate(currentPose.getMeasureY().in(Inches)))
					);
					
					double maxAcceleration = 96;
					double maxDeceleration = 96;
					double maxVelocityToStop = Math.sqrt(2 * maxDeceleration * inchesRemaining);
					double currentVelocity = Swerve.this.getLinearVelocity().in(InchesPerSecond);
					double desiredVelocity = currentVelocity < maxVelocityToStop
						? Math.min(currentVelocity + maxAcceleration * 0.02, maxVelocityToStop)
						: Math.max(currentVelocity - maxDeceleration * 0.02, maxVelocityToStop);
					
					point = point.div(
						point.getNorm() /
						InchesPerSecond.of(desiredVelocity).in(MetersPerSecond)
					);
					
					point = new Translation2d(
						InchesPerSecond.of(desiredVelocity).in(MetersPerSecond),
						point.getAngle()
					);
					
					System.out.printf("linear velocity: %.2f\n", MetersPerSecond.of(point.getNorm()).in(InchesPerSecond));
					
					boolean needsInverting = DriverStation.Alliance.Red ==
						DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
					
					if (needsInverting) point = point.times(-1);
					
					Swerve.this.applyChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
						MetersPerSecond.of(point.getX()),
						MetersPerSecond.of(point.getY()),
						DegreesPerSecond.of(Math.min(thetaFeed, this.MAX_ANGULAR_VELOCITY.in(DegreesPerSecond)))
					), Rotation2d.fromDegrees(Swerve.this.gyro.getRotation().in(Degrees))));
					
				}
				
				@Override
				public boolean isFinished() {
					
					Pose2d relativePose = currentPose.relativeTo(desiredPose);
					Distance linearDistance = Meters.of(relativePose.getTranslation().getNorm());
					
					if (linearDistance.lt(Inches.of(0))) linearDistance = linearDistance.times(-1);
					
					return (
						linearDistance.lte(distanceTolerance) &&
						currentPose.getRotation().getMeasure().isNear(desiredPose.getRotation().getMeasure(), angularTolerance)
					);
					
				}
				
				@Override
				public void end(boolean interrupted) {
					
					Swerve.this.odometry.removeDisplaySetpoint();
					Swerve.this.odometry.vision.resetAprilTagFilter();
					Swerve.this.applyChassisSpeeds(new ChassisSpeeds(0, 0, 0));
					
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
		
		public Command goToPosition2(Pose2d pose) {
			
			double linearKp = 0.000000002;
			LinearVelocity maxVelocity = InchesPerSecond.of(80);
			LinearAcceleration maxAcceleration = InchesPerSecond.per(Second).of(160);
			
			TrajectoryConfig config = new TrajectoryConfig(
				maxVelocity.in(MetersPerSecond),
				maxAcceleration.in(MetersPerSecondPerSecond)
			);

			return new DeferredCommand(() -> {
				
				Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
					List.of(Swerve.this.odometry.getPose(), pose),
					config
				);
				
				return new SwerveControllerCommand(
					trajectory,
					Swerve.this.odometry::getPose,
					Swerve.this.kinematics,
					new PIDController(linearKp, 0, 0),
					new PIDController(linearKp, 0, 0),
					new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(
							180,
							90
					)),
					Swerve.this::applyModuleStates,
					Swerve.this
				).andThen(this.stop());
				
			}, Set.of(Swerve.this));
			
		}
	
	}
	
}
