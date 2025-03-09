// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.configuration.ReefBranch;
import frc.robot.configuration.RobotDimensions;
import frc.robot.configuration.SwerveModuleConfiguration;
import frc.robot.devicewrappers.LimelightHelpers;
import frc.robot.devicewrappers.RaptorsNavX;
import frc.robot.util.*;

import java.util.*;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Stream;

import static edu.wpi.first.units.Units.*;

public class Swerve extends SubsystemBase {
	
	protected static final String FRONT_LIMELIGHT_NAME = "limelight-front";
	
	protected static final String REAR_LIMELIGHT_NAME = "limelight-rear";
	
	protected static final int[] REEF_TAG_IDS = {
		6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22
	};
	
	protected static final String[] LIMELIGHTS = new String[] {
		FRONT_LIMELIGHT_NAME,
		REAR_LIMELIGHT_NAME
	};
	
	public final SwerveModule[] modules;
	
	protected final PIDController headingPIDController;
	
	protected final RaptorsNavX gyro;
	
	protected final SwerveDriveKinematics kinematics;
	
//	protected final SwerveDriveOdometry odometry;
	
	protected final SwerveDrivePoseEstimator poseEstimator;
	
	protected final Field2d field;
	
	protected Pose2d mostRecentFrontTag;
	
	protected Pose2d mostRecentRearTag;
	
	protected AprilTag mostRecentReefTag;
	
	protected AprilTag nearestReefTag;
	
	protected boolean isHeadingLockEnabled;
	
	protected ChassisSpeeds chassisSpeeds;
	
	public final Swerve.Commands commands;
	
	public Swerve() {
		
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
		this.poseEstimator = new SwerveDrivePoseEstimator(
			this.kinematics,
			new Rotation2d(this.getFieldRelativeHeading()),
			this.getModulePositions(),
			new Pose2d(),
			VecBuilder.fill(1, 1, 0.7),
			VecBuilder.fill(5, 5, 999999)
		);
		this.field = new Field2d();
		this.isHeadingLockEnabled = false;
		this.commands = new Swerve.Commands();
		this.chassisSpeeds = new ChassisSpeeds(0, 0, 0);
		
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
		SmartDashboard.putData("Field Odometry", this.field);
		
	}
	
	protected Stream<SwerveModule> getModuleStream() {
		
		return Stream.of(this.modules);
		
	}
	
	protected SwerveModulePosition[] getModulePositions() {
		
		return this.getModuleStream()
			.map(SwerveModule::getPosition)
			.toArray(SwerveModulePosition[]::new);
		
	}
	
	protected void applyModuleStates(SwerveModuleState[] moduleStates) {
		
		this.getModuleStream().forEach(module -> {
			module.updateModuleState(moduleStates[module.getID()]);
		});
		
	}
	
	public void stop() {
		
		this.applyChassisSpeeds(new ChassisSpeeds(0, 0, 0), false);
		
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
	
	public boolean hasReefAprilTagLock() {
		
		return this.mostRecentReefTag != null;
		
	}
	
	public void applyChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean fieldRelative) {
		
		if (fieldRelative) {
			
			chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
				chassisSpeeds,
				Rotation2d.fromDegrees(this.gyro.getRotation().in(Degrees))
			);
			
		}

//		// Poll the current state of the heading lock.
//		boolean wasHeadingLockEnabled = this.isHeadingLockEnabled;
//
//		// Enable the heading lock if we are not receiving any rotational input,
//		// otherwise, disable it (if we *are* receiving rotational input).
//		this.isHeadingLockEnabled = Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= 0;
//
//		// Check for a rising edge of the heading lock state.
//		boolean didHeadingLockBecomeEnabled = (
//			!wasHeadingLockEnabled &&
//			this.isHeadingLockEnabled
//		);
//
//		// If the heading lock *became* active...
//		if (didHeadingLockBecomeEnabled) {
//
//			// Update the heading setpoint to the heading we've rotated to while
//			// the heading lock was disabled.
//			this.headingPIDController.setSetpoint(
//				this.getFieldRelativeHeading().in(Degrees)
//			);
//
//		}

//		chassisSpeeds.omegaRadiansPerSecond *= 1.5;
		
		// Update the chassis speeds.
		this.chassisSpeeds = chassisSpeeds;
		
	}
	
	public void setFieldRelativeHeadingSetpoint(Angle heading) {
		
		Pose2d existingPose = this.poseEstimator.getEstimatedPosition();
		
		this.headingPIDController.setSetpoint(heading.in(Degrees));
		
		this.poseEstimator.resetPose(existingPose);
//		this.odometry.resetPosition(
//			new Rotation2d(0),
//			this.getModulePositions(),
//			this.currentPose
//		);
		
	}
	
	@Override
	public void periodic() {
		
//		Angle heading = this.getFieldRelativeHeading();
		
		this.updatePose();
		
//		double headingPIDOutput =
//			this.headingPIDController.calculate(heading.in(Degrees));
//
//		this.isHeadingLockEnabled = false;
//
//		ChassisSpeeds newChassisSpeeds = new ChassisSpeeds(
//			this.chassisSpeeds.vxMetersPerSecond,
//			this.chassisSpeeds.vyMetersPerSecond,
//			this.isHeadingLockEnabled ? headingPIDOutput : this.chassisSpeeds.omegaRadiansPerSecond
//		);
		
		SwerveModuleState[] newModuleStates =
			this.kinematics.toSwerveModuleStates(this.chassisSpeeds);

		this.applyModuleStates(
			newModuleStates
		);
		
	}
	
	public void updatePose() {
		
		this.poseEstimator.update(
			new Rotation2d(this.getFieldRelativeHeading()),
			this.getModulePositions()
		);
		
//		this.currentPose = this.odometry.update(
//			new Rotation2d(this.getFieldRelativeHeading()),
//			this.getModulePositions()
//		);
		
		double limelightYaw = this.getFieldRelativeHeading().in(Degrees);
		
		if (
			DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) ==
				DriverStation.Alliance.Red
		) limelightYaw += 180;
		
		for (String limelightName: LIMELIGHTS) {
			
			LimelightHelpers.SetRobotOrientation(
				limelightName,
				limelightYaw,
				0,
				0,
				0,
				0,
				0
			);
			
		}
		
		LimelightHelpers.PoseEstimate frontPoseEstimate =
			LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(FRONT_LIMELIGHT_NAME);
		LimelightHelpers.PoseEstimate rearPoseEstimate =
			LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(FRONT_LIMELIGHT_NAME);
		
		if (
			frontPoseEstimate != null &&
			frontPoseEstimate.tagCount > 0
		) {
			
			int frontTagID = frontPoseEstimate.rawFiducials[0].id;
			this.updateMostRecentReefTag(AprilTagHelper.getAprilTag(frontTagID));
			
			this.poseEstimator.addVisionMeasurement(
				frontPoseEstimate.pose,
				frontPoseEstimate.timestampSeconds,
				VecBuilder.fill(0.5, 0.5, 30)
			);
			
		}
		
		if (
			rearPoseEstimate != null &&
			rearPoseEstimate.tagCount > 0
		) {
			
			int rearTagID = rearPoseEstimate.rawFiducials[0].id;
			this.updateMostRecentReefTag(AprilTagHelper.getAprilTag(rearTagID));
			
			this.poseEstimator.addVisionMeasurement(
				rearPoseEstimate.pose,
				rearPoseEstimate.timestampSeconds,
				VecBuilder.fill(0.5, 0.5, 30)
			);
			
		}
		
//		List<LimelightHelpers.PoseEstimate> poseEstimates = Stream.of(LIMELIGHTS)
//			.map(LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2)
//			.filter(x -> x != null && x.tagCount > 0 && x.avgTagDist < 2.5)
//			.sorted(Comparator.comparingDouble(x -> x.avgTagDist))
//			.toList();
//
//		if (!poseEstimates.isEmpty()) {
//
//			LimelightHelpers.PoseEstimate poseEstimate = poseEstimates.get(0);
//
//			this.poseEstimator.addVisionMeasurement(
//				poseEstimate.pose,
//				poseEstimate.timestampSeconds,
//				VecBuilder.fill(0.5, 0.5, 30)
//			);
//
//		}
		
		this.nearestReefTag = VirtualField.getNearestReefAprilTag(
			this.poseEstimator.getEstimatedPosition().getTranslation()
		);
		
		this.field.setRobotPose(this.poseEstimator.getEstimatedPosition());
		this.field.getObject("nearest-april-tag")
			.setPose(this.nearestReefTag.pose.toPose2d());
		
	}
	
	protected void updateMostRecentReefTag(AprilTag tag) {
		
//		System.out.println("updating most recent reef tag: " + tag.ID);
	
		if (Arrays.stream(Swerve.REEF_TAG_IDS).anyMatch((id) -> id == tag.ID)) {
			
			this.mostRecentReefTag = tag;
//			this.field.getObject("most-recent-reef-tag")
//				.setPose(RobotPoseHelper.getCenteredRobotPoseForReefAprilTag(tag.ID));
//			this.field.getObject("most-recent-reef-tag-left")
//				.setPose(RobotPoseHelper.getBranchRobotPoseForReefAprilTag(tag.ID, ReefBranch.LEFT));
//			this.field.getObject("most-recent-reef-tag-right")
//				.setPose(RobotPoseHelper.getBranchRobotPoseForReefAprilTag(tag.ID, ReefBranch.RIGHT));
			
		}
	
	}
	
	public AprilTag getNearestReefAprilTag() {
		
		return VirtualField.getNearestReefAprilTag(
			Swerve.this.poseEstimator.getEstimatedPosition().getTranslation()
		);
		
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
			() -> this.headingPIDController.getSetpoint(),
			(double headingDegrees) -> this.setFieldRelativeHeadingSetpoint(Degrees.of(headingDegrees))
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
				"Chassis Speeds (Rotation in degrees per second)",
				() -> RadiansPerSecond.of(this.chassisSpeeds.omegaRadiansPerSecond).in(DegreesPerSecond),
				null
			);
			
			builder.addIntegerProperty(
				"Most Recent Reef Tag",
				() -> this.mostRecentReefTag == null ? -1 : this.mostRecentReefTag.ID,
				(id) -> this.mostRecentReefTag = AprilTagHelper.getAprilTag((int) id)
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
		
		public Command drive(
			Supplier<Point> xyInchesPerSecond,
			DoubleSupplier rotationDegreesPerSecond,
			boolean fieldRelative
		) {
			
			return Swerve.this.run(() -> {
				
				Point xy = xyInchesPerSecond.get();
				
				Swerve.this.applyChassisSpeeds(new ChassisSpeeds(
					InchesPerSecond.of(xy.x),
					InchesPerSecond.of(xy.y),
					DegreesPerSecond.of(rotationDegreesPerSecond.getAsDouble())
				), fieldRelative);
				
			});
			
		}
		
		public Command goToNearestReefPosition(ReefBranch branch) {
			
			return this.goToPosition(() -> RobotPoseHelper.getBranchRobotPoseForReefAprilTag(
				Swerve.this.nearestReefTag.ID,
				branch
			));
			
		}
		
		public Command waitUntilAtNearestReefPosition(ReefBranch branch) {
			
			return this.waitUntilAtPosition(() ->
				RobotPoseHelper.getBranchRobotPoseForReefAprilTag(
					Swerve.this.nearestReefTag.ID,
					branch
				),
				Inches.of(2),
				Degrees.of(5)
			);
			
		}
		
		public Command goToPosition(Supplier<Pose2d> poseSupplier) {
			
			Command command = new Command() {
				
				final double LINEAR_KP = 5;
				
				final double ANGULAR_KP = 5;
				
				final LinearVelocity MAX_LINEAR_VELOCITY = InchesPerSecond.of(15);
				
				final AngularVelocity MAX_ANGULAR_VELOCITY = DegreesPerSecond.of(45);
				
				final PIDController xController =
					new PIDController(LINEAR_KP, 0, 0);
				
				final PIDController yController =
					new PIDController(LINEAR_KP, 0, 0);
				
				final PIDController thetaController =
					new PIDController(ANGULAR_KP, 0, 0);
				
				@Override
				public void initialize() {
					
					Pose2d pose = poseSupplier.get();
					
					Swerve.this.field.getObject("setpoint").setPose(pose);
					
					this.thetaController.enableContinuousInput(-180, 180);
					
					xController.setSetpoint(pose.getMeasureX().in(Inches));
					yController.setSetpoint(pose.getMeasureY().in(Inches));
					thetaController.setSetpoint(pose.getRotation().getMeasure().in(Degrees));
				
				}
				
				@Override
				public void execute() {
					
					Pose2d currentPose = Swerve.this.poseEstimator
						.getEstimatedPosition();
					
					double xFeed = xController.calculate(currentPose.getMeasureX().in(Inches));
					double yFeed = yController.calculate(currentPose.getMeasureY().in(Inches));
					double thetaFeed = thetaController.calculate(currentPose.getRotation().getDegrees());
					
					if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
						
						xFeed *= -1;
						yFeed *= -1;
						
					}
					
					Swerve.this.applyChassisSpeeds(new ChassisSpeeds(
						InchesPerSecond.of(Math.min(xFeed, this.MAX_LINEAR_VELOCITY.in(InchesPerSecond))),
						InchesPerSecond.of(Math.min(yFeed, this.MAX_LINEAR_VELOCITY.in(InchesPerSecond))),
						DegreesPerSecond.of(Math.min(thetaFeed, this.MAX_ANGULAR_VELOCITY.in(DegreesPerSecond)))
					), true);
					
				}
				
				@Override
				public void end(boolean interrupted) {
					
					Swerve.this.field.getObject("setpoint").setPoses();
					
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
				Pose2d currentPose = Swerve.this.poseEstimator.getEstimatedPosition();
				
				return (
					currentPose.getMeasureX().isNear(desiredPose.getMeasureX(), distanceTolerance) &&
					currentPose.getMeasureY().isNear(desiredPose.getMeasureY(), distanceTolerance) &&
					currentPose.getRotation().getMeasure().isNear(desiredPose.getRotation().getMeasure(), angularTolerance)
				);
				
			});
			
		}
	
	}
	
}
