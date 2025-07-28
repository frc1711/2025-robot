package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.configuration.ReefAlignment;
import frc.robot.configuration.RobotDimensions;

import java.util.List;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;

/**
 * A builder class for creating and manipulating robot poses.
 */
public class RobotPoseBuilder {

	/**
	 * The underlying pose representing the current 'state' of the pose being
	 * built.
	 */
	protected final Pose2d pose;

	/**
	 * Initializes a new RobotPoseBuilder with the given pose.
	 *
	 * @param pose The initial pose to build upon.
	 */
	protected RobotPoseBuilder(Pose2d pose) {
	
		this.pose = pose;
	
	}

	/**
	 * Returns a new RobotPoseBuilder with the given pose.
	 *
	 * @param pose The pose to initialize the builder with.
	 * @return A RobotPoseBuilder with the given pose.
	 */
	public static RobotPoseBuilder fromPose(Pose2d pose) {
		
		return new RobotPoseBuilder(pose);
		
	}

	/**
	 * Returns a RobotPoseBuilder representing a pose at the center of the
	 * virtual field, facing the positive X direction (0 degrees).
	 *
	 * @return A RobotPoseBuilder representing a pose at the center of the
	 * virtual field.
	 */
	public static RobotPoseBuilder fromCenterFieldPose() {
		
		return new RobotPoseBuilder(new Pose2d(
			VirtualField.FIELD_LENGTH.div(2),
			VirtualField.FIELD_WIDTH.div(2),
			Rotation2d.kZero
		));
		
	}

	/**
	 * Returns a RobotPoseBuilder representing the pose of the AprilTag with
	 * the given ID.
	 *
	 * @param tagID The ID of the AprilTag to get the pose for.
	 * @return A RobotPoseBuilder representing the pose of the AprilTag with
	 * the given ID.
	 */
	public static RobotPoseBuilder getAprilTagPose(int tagID) {

		return new RobotPoseBuilder(
			AprilTagHelper.getAprilTag(tagID).pose.toPose2d()
		);

	}

	/**
	 * Returns a pose facing the AprilTag with the given ID, with an additional
	 * offset from the robot's face to the tag.
	 *
	 * @param tagID The ID of the AprilTag to face.
	 * @param extraOffset The additional offset from the robot's face to the tag.
	 * @return A RobotPoseBuilder representing a pose facing the AprilTag with
	 * the given ID and an additional offset.
	 */
	public static RobotPoseBuilder getAprilTagFacingPose(
		int tagID,
		Distance extraOffset
	) {

		return RobotPoseBuilder.getAprilTagPose(tagID)
			.withRobotRelativeHeading(Rotation2d.k180deg)
			.withRobotRelativeTranslation(new Translation2d(
				RobotDimensions.ROBOT_LENGTH.div(2).minus(extraOffset).times(-1),
				Inches.of(0)
			));

	}

	/**
	 * Returns a pose facing the AprilTag with the given ID.
	 *
	 * @param tagID The ID of the AprilTag to face.
	 * @return A RobotPoseBuilder representing a pose facing the AprilTag with
	 * the given ID.
	 */
	public static RobotPoseBuilder getAprilTagFacingPose(int tagID) {

		return RobotPoseBuilder.getAprilTagFacingPose(
			tagID,
			Inches.of(0)
		);

	}

	/**
	 * Returns a pose for scoring on the reef, facing the AprilTag with the
	 * given ID, with the robot positioned relative to the tag based on the
	 * specified alignment.
	 *
	 * @param tagID The ID of the AprilTag to face.
	 * @param alignment The reef branch to align the mailbox with, or CENTER if
	 * the robot should be centered on the tag.
	 * @return A RobotPoseBuilder representing a pose for scoring on the reef.
	 */
	public static RobotPoseBuilder getReefScoringPose(
		int tagID,
		ReefAlignment alignment
	) {

		return RobotPoseBuilder.getAprilTagFacingPose(tagID)
			.withRobotRelativeTranslation(alignment.equals(ReefAlignment.CENTER)
				? Translation2d.kZero
				: new Translation2d(
					Inches.of(0),
					RobotDimensions.REEF_BRANCH_SEPARATION_DISTANCE.div(2)
						.times(alignment.equals(ReefAlignment.LEFT) ? 1 : -1)
						.minus(RobotDimensions.MAILBOX_LR_OFFSET_TO_ROBOT_CENTER)
				)
			);
		
	}

	/**
	 * Returns a pose for 'calibrating' the vision camera on the reef in front
	 * of the given tag and with the specified alignment.
	 *
	 * @param tagID The ID of the AprilTag to face.
	 * @param alignment The reef branch to align the mailbox with, or CENTER if
	 * the robot should be centered on the tag.
	 * @return A RobotPoseBuilder representing a pose for calibrating the reef
	 * vision camera.
	 */
	public static RobotPoseBuilder getReefCalibrationPose(
		int tagID,
		ReefAlignment alignment
	) {
		
		return RobotPoseBuilder.getReefScoringPose(tagID, alignment)
			.withRobotRelativeTranslation(new Translation2d(
				Inches.of(-30),
				Inches.of(0)
			));
		
	}

	/**
	 * Returns a RobotPoseBuilder representing a pose for loading coral from the
	 * nearest coral loading station.
	 *
	 * @param tagID The ID of the AprilTag representing the coral loading
	 * station.
	 * @return A RobotPoseBuilder representing a pose for loading coral from the
	 * nearest coral loading station.
	 */
	public static RobotPoseBuilder getCoralStationLoadingPose(int tagID) {

		return RobotPoseBuilder.getAprilTagFacingPose(tagID)
			.withRobotRelativeTranslation(new Translation2d(
				Inches.of(0),
				Feet.of(-1)
			));
		
	}

	/**
	 * Returns a RobotPoseBuilder representing a pose for calibrating the
	 * coral station loading pose, which is the same as the loading pose but
	 * offset by 24 inches in the positive X direction (away from the tag).
	 *
	 * @param tagID The ID of the AprilTag representing the coral loading
	 * station.
	 * @return A RobotPoseBuilder representing a pose for calibrating the coral
	 * station loading pose.
	 */
	public static RobotPoseBuilder getCoralStationCalibrationPose(int tagID) {
		
		return RobotPoseBuilder.getCoralStationLoadingPose(tagID)
			.withRobotRelativeTranslation(new Translation2d(
				Inches.of(-24),
				Inches.of(0)
			));
		
	}

	/**
	 * Returns a modified version of the current RobotPoseBuilder pose, having
	 * applied the given transformation to it under a 'blue out' UCS (where
	 * positive X faces outwards from the blue alliance wall).
	 *
	 * @param translation The 'blue out' relative translation to apply to the
	 * current pose.
	 * @return A new RobotPoseBuilder with the applied translation.
	 */
	public RobotPoseBuilder withBlueOutRelativeTranslation(
		Translation2d translation
	) {
		
		return new RobotPoseBuilder(this.pose.plus(new Transform2d(
			translation.rotateBy(this.pose.getRotation().times(-1)),
			Rotation2d.kZero
		)));
		
	}

	/**
	 * Returns a modified version of the current RobotPoseBuilder pose, having
	 * applied the given transformation to it under a 'field relative' UCS
	 * (where positive X faces outwards from the specified alliance's wall).
	 *
	 * @param alliance The alliance to use for determining the field-relative
	 * coordinate system.
	 * @param translation The 'field relative' translation to apply to the
	 * current pose.
	 * @return A new RobotPoseBuilder with the applied translation.
	 */
	public RobotPoseBuilder withFieldRelativeTranslation(
		DriverStation.Alliance alliance,
		Translation2d translation
	) {
		
		boolean shouldInvert = alliance == DriverStation.Alliance.Red;
		
		return this.withBlueOutRelativeTranslation(
			translation.times(shouldInvert ? -1 : 1)
		);
		
	}

	/**
	 * Returns a modified version of the current RobotPoseBuilder pose, having
	 * applied the given transformation to it under a 'field relative' UCS
	 * (where positive X faces outwards from the alliance wall, as informed by
	 * the FMS).
	 *
	 * @param translation The 'field relative' translation to apply to the
	 * current pose.
	 * @return A new RobotPoseBuilder with the applied translation.
	 */
	public RobotPoseBuilder withFieldRelativeTranslation(
		Translation2d translation
	) {
		
		return this.withFieldRelativeTranslation(
			DriverStation.getAlliance().orElse(DriverStation.Alliance.Red),
			translation
		);
		
	}

	/**
	 * Returns a modified version of the current RobotPoseBuilder pose, having
	 * applied the given transformation to it under a 'robot relative' UCS
	 * (where positive X faces in the direction of the robot's current heading).
	 *
	 * @param translation The 'robot relative' translation to apply to the
	 * current pose.
	 * @return A new RobotPoseBuilder with the applied translation.
	 */
	public RobotPoseBuilder withRobotRelativeTranslation(
		Translation2d translation
	) {
		
		return new RobotPoseBuilder(this.pose.plus(new Transform2d(
			translation,
			Rotation2d.kZero
		)));
		
	}

	/**
	 * Returns a modified version of the current RobotPoseBuilder pose, having
	 * set the heading of the pose to the specified heading under a 'blue out'
	 * UCS (where positive X faces outwards from the blue alliance wall).
	 *
	 * @param heading The 'blue out' relative heading to set for the current
	 * pose.
	 * @return A new RobotPoseBuilder with the applied heading.
	 */
	public RobotPoseBuilder withBlueOutHeading(Rotation2d heading) {
		
		return new RobotPoseBuilder(this.pose.plus(new Transform2d(
			new Translation2d(),
			heading
		)));
		
	}

	/**
	 * Returns a modified version of the current RobotPoseBuilder pose, having
	 * set the heading of the pose to the specified heading under a 'blue out'
	 * UCS (where positive X faces outwards from the blue alliance wall).
	 *
	 * @param heading The 'blue out' relative heading to set for the current
	 * pose.
	 * @return A new RobotPoseBuilder with the applied heading.
	 */
	public RobotPoseBuilder withBlueOutHeading(Angle heading) {
		
		return this.withBlueOutHeading(new Rotation2d(heading));
		
	}

	/**
	 * Returns a modified version of the current RobotPoseBuilder pose, having
	 * set the heading of the pose to the specified heading under a 'field
	 * relative' UCS (where positive X faces outwards from the specified
	 * alliance's wall).
	 *
	 * @param alliance The alliance to use for determining the field-relative
	 * coordinate system.
	 * @param heading The 'field relative' heading to set for the current pose.
	 * @return A new RobotPoseBuilder with the applied heading.
	 */
	public RobotPoseBuilder withFieldRelativeHeading(
		DriverStation.Alliance alliance,
		Rotation2d heading
	) {
		
		boolean shouldInvert = alliance == DriverStation.Alliance.Red;
		
		return this.withBlueOutHeading(heading.times(shouldInvert ? -1 : 1));
		
	}

	/**
	 * Returns a modified version of the current RobotPoseBuilder pose, having
	 * set the heading of the pose to the specified heading under a 'field
	 * relative' UCS (where positive X faces outwards from the alliance wall,
	 * as informed by the FMS).
	 *
	 * @param alliance The alliance to use for determining the field-relative
	 * coordinate system.
	 * @param heading The 'field relative' heading to set for the current pose.
	 * @return A new RobotPoseBuilder with the applied heading.
	 */
	public RobotPoseBuilder withFieldRelativeHeading(
		DriverStation.Alliance alliance,
		Angle heading
	) {
		
		return this.withFieldRelativeHeading(alliance, new Rotation2d(heading));
		
	}

	/**
	 * Returns a modified version of the current RobotPoseBuilder pose, having
	 * set the heading of the pose to the specified heading under a 'field
	 * relative' UCS (where positive X faces outwards from the alliance wall,
	 * as informed by the FMS).
	 *
	 * @param heading The 'field relative' heading to set for the current pose.
	 * @return A new RobotPoseBuilder with the applied heading.
	 */
	public RobotPoseBuilder withFieldRelativeHeading(Rotation2d heading) {
		
		return this.withFieldRelativeHeading(
			DriverStation.getAlliance().orElse(DriverStation.Alliance.Red),
			heading
		);
		
	}

	/**
	 * Returns a modified version of the current RobotPoseBuilder pose, having
	 * set the heading of the pose to the specified heading under a 'field
	 * relative' UCS (where positive X faces outwards from the alliance wall,
	 * as informed by the FMS).
	 *
	 * @param heading The 'field relative' heading to set for the current pose.
	 * @return A new RobotPoseBuilder with the applied heading.
	 */
	public RobotPoseBuilder withFieldRelativeHeading(Angle heading) {
		
		return this.withFieldRelativeHeading(new Rotation2d(heading));
		
	}
	
	public RobotPoseBuilder withRobotRelativeHeading(Rotation2d heading) {
		
		return new RobotPoseBuilder(new Pose2d(
			this.pose.getTranslation(),
			this.pose.getRotation().plus(heading)
		));
		
	}

	/**
	 * Returns a modified version of the current RobotPoseBuilder pose, having
	 * set the heading of the pose to the specified heading under a 'robot
	 * relative' UCS (where positive X faces in the direction of the robot's
	 * current heading).
	 *
	 * @param heading The 'robot relative' heading to set for the current pose.
	 * @return A new RobotPoseBuilder with the applied heading.
	 */
	public RobotPoseBuilder withRobotRelativeHeading(Angle heading) {
		
		return this.withRobotRelativeHeading(new Rotation2d(heading));
		
	}

	/**
	 * Returns the current pose represented by this RobotPoseBuilder.
	 *
	 * @return The current Pose2d represented by this RobotPoseBuilder.
	 */
	public Pose2d toPose() {
		
		return this.pose;
		
	}
	
}
