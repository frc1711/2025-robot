package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.configuration.ReefAlignment;
import frc.robot.configuration.RobotDimensions;

import java.util.List;

import static edu.wpi.first.units.Units.Inches;

public class RobotPoseBuilder {
	
	protected final Pose2d pose;
	
	protected RobotPoseBuilder(Pose2d pose) {
	
		this.pose = pose;
	
	}
	
	public static RobotPoseBuilder fromPose(Pose2d pose) {
		
		return new RobotPoseBuilder(pose);
		
	}
	
	public static RobotPoseBuilder fromCenterFieldPose() {
		
		return new RobotPoseBuilder(new Pose2d(
			VirtualField.FIELD_LENGTH.div(2),
			VirtualField.FIELD_WIDTH.div(2),
			Rotation2d.kZero
		));
		
	}
	
	public static RobotPoseBuilder getReefScoringPose(
		int tagID,
		ReefAlignment alignment
	) {
		
		Pose2d tagPose = AprilTagHelper.getAprilTag(tagID).pose.toPose2d();
		
		RobotPoseBuilder result = new RobotPoseBuilder(tagPose)
			.withRobotRelativeHeading(Rotation2d.k180deg)
			.withRobotRelativeTranslation(new Translation2d(
//				RobotDimensions.ROBOT_LENGTH.div(2).times(-1),
				RobotDimensions.ROBOT_LENGTH.div(2).minus(Inches.of(1.5)).times(-1),
				Inches.of(0)
			));
		
		result = switch (alignment) {
			
			case CENTER -> result;
			
			case LEFT -> result.withRobotRelativeTranslation(new Translation2d(
				Inches.of(0),
				RobotDimensions.REEF_BRANCH_SEPARATION_DISTANCE.div(2)
					.minus(RobotDimensions.MAILBOX_LR_OFFSET_TO_ROBOT_CENTER)
			));
			
			case RIGHT -> result.withRobotRelativeTranslation(new Translation2d(
				Inches.of(0),
				RobotDimensions.REEF_BRANCH_SEPARATION_DISTANCE.div(2).times(-1)
					.minus(RobotDimensions.MAILBOX_LR_OFFSET_TO_ROBOT_CENTER)
			));
			
		};
		
		// TODO clean this up
//		if (List.of(7, 10, 18, 21).contains(tagID)) {
//
//			result = result.withRobotRelativeTranslation(new Translation2d(
//				Inches.of(0),
//				Inches.of(-2)
//			));
//
//		}
		
		return result;
		
//		return new RobotPoseBuilder(switch (alignment) {
//			case LEFT -> RobotPoseHelper.getBranchRobotPoseForReefAprilTag(tagID, ReefAlignment.LEFT, ReefLevel.L2);
//			case CENTER -> RobotPoseHelper.getCenteredRobotPoseForReefAprilTag(tagID, ReefLevel.L2);
//			case RIGHT -> RobotPoseHelper.getBranchRobotPoseForReefAprilTag(tagID, ReefAlignment.RIGHT, ReefLevel.L2);
//		});
		
	}
	
	public static RobotPoseBuilder getReefCalibrationPose(int tagID) {
		
		return RobotPoseBuilder.getReefScoringPose(tagID, ReefAlignment.CENTER)
			.withRobotRelativeTranslation(new Translation2d(
				Inches.of(-30),
				Inches.of(-6)
			));
		
	}
	
	public static RobotPoseBuilder getCoralStationLoadingPose(int tagID) {
		
		Pose2d tagPose = AprilTagHelper.getAprilTag(tagID).pose.toPose2d();
		
		return new RobotPoseBuilder(tagPose)
			.withRobotRelativeTranslation(new Translation2d(
				RobotDimensions.ROBOT_LENGTH.div(2).minus(Inches.of(1.5)),
				Inches.of(-12)
			));
		
	}
	
	public static RobotPoseBuilder getCoralStationCalibrationPose(int tagID) {
		
		return RobotPoseBuilder.getCoralStationLoadingPose(tagID)
			.withRobotRelativeTranslation(new Translation2d(
				Inches.of(24),
				Inches.of(0)
			));
		
	}
	
	public RobotPoseBuilder withBlueOutRelativeTranslation(
		Translation2d translation
	) {
		
		return new RobotPoseBuilder(this.pose.plus(new Transform2d(
			translation.rotateBy(this.pose.getRotation().times(-1)),
			Rotation2d.kZero
		)));
		
	}
	
	public RobotPoseBuilder withFieldRelativeTranslation(
		DriverStation.Alliance alliance,
		Translation2d translation
	) {
		
		boolean shouldInvert = alliance == DriverStation.Alliance.Red;
		
		return this.withBlueOutRelativeTranslation(
			translation.times(shouldInvert ? -1 : 1)
		);
		
	}
	
	public RobotPoseBuilder withFieldRelativeTranslation(
		Translation2d translation
	) {
		
		return this.withFieldRelativeTranslation(
			DriverStation.getAlliance().orElse(DriverStation.Alliance.Red),
			translation
		);
		
	}
	
	public RobotPoseBuilder withRobotRelativeTranslation(
		Translation2d translation
	) {
		
		return new RobotPoseBuilder(this.pose.plus(new Transform2d(
			translation,
			Rotation2d.kZero
		)));
		
	}
	
	public RobotPoseBuilder withBlueOutHeading(Rotation2d heading) {
		
		return new RobotPoseBuilder(this.pose.plus(new Transform2d(
			new Translation2d(),
			heading
		)));
		
	}
	
	public RobotPoseBuilder withBlueOutHeading(Angle heading) {
		
		return this.withBlueOutHeading(new Rotation2d(heading));
		
	}
	
	public RobotPoseBuilder withFieldRelativeHeading(
		DriverStation.Alliance alliance,
		Rotation2d heading
	) {
		
		boolean shouldInvert = alliance == DriverStation.Alliance.Red;
		
		return this.withBlueOutHeading(heading.times(shouldInvert ? -1 : 1));
		
	}
	
	public RobotPoseBuilder withFieldRelativeHeading(
		DriverStation.Alliance alliance,
		Angle heading
	) {
		
		return this.withFieldRelativeHeading(alliance, new Rotation2d(heading));
		
	}
	
	public RobotPoseBuilder withFieldRelativeHeading(Rotation2d heading) {
		
		return this.withFieldRelativeHeading(
			DriverStation.getAlliance().orElse(DriverStation.Alliance.Red),
			heading
		);
		
	}
	
	public RobotPoseBuilder withFieldRelativeHeading(Angle heading) {
		
		return this.withFieldRelativeHeading(new Rotation2d(heading));
		
	}
	
	public RobotPoseBuilder withRobotRelativeHeading(Rotation2d heading) {
		
		return new RobotPoseBuilder(new Pose2d(
			this.pose.getTranslation(),
			this.pose.getRotation().plus(heading)
		));
		
	}
	
	public RobotPoseBuilder withRobotRelativeHeading(Angle heading) {
		
		return this.withRobotRelativeHeading(new Rotation2d(heading));
		
	}
	
	public Pose2d toPose() {
		
		return this.pose;
		
	}
	
}
