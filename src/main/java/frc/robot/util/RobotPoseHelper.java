package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.configuration.ReefBranch;
import frc.robot.configuration.RobotDimensions;

import static edu.wpi.first.units.Units.Inches;

public class RobotPoseHelper {
	
	protected RobotPoseHelper() {}
	
	public static Pose2d translateAtAngle(Pose2d pose, Rotation2d angle, Distance distance) {
	
		return pose.plus(new Transform2d(
			new Translation2d(distance, Inches.of(0)),
			angle
		));
	
	}
	
	public static Pose2d translateAtOrthogonalAngle(Pose2d pose, Rotation2d angle, Distance distance) {
		
		return pose.plus(new Transform2d(
			new Translation2d(Inches.of(0), distance),
			angle
		));
		
	}
	
	public static Pose2d setPoseRotationInPlace(Pose2d pose, Rotation2d rotation) {
		
		return new Pose2d(
			pose.getTranslation(),
			rotation
		);
		
	}
	
	public static Pose2d getCenteredRobotPoseForReefAprilTag(int tagID) {
		
		Pose2d tagPose = AprilTagHelper.getInstance()
			.getAprilTag(tagID)
			.pose
			.toPose2d();
		
		return setPoseRotationInPlace(
			translateAtAngle(
				tagPose,
				tagPose.getRotation(),
				RobotDimensions.ROBOT_LENGTH.div(2)
			),
			tagPose.getRotation().plus(Rotation2d.fromRotations(0.5))
		);
	
	}
	
	public static Pose2d getBranchRobotPoseForReefAprilTag(int tagID, ReefBranch branch) {
		
		Pose2d centeredPose =
			RobotPoseHelper.getCenteredRobotPoseForReefAprilTag(tagID);
		Distance shiftAmount = RobotDimensions.REEF_BRANCH_SEPARATION_DISTANCE
			.div(2)
			.minus(RobotDimensions.MAILBOX_LR_OFFSET_TO_ROBOT_CENTER);
		
		if (branch == ReefBranch.RIGHT) {

			shiftAmount = shiftAmount.minus(
				RobotDimensions.REEF_BRANCH_SEPARATION_DISTANCE
			).plus(Inches.of(2.5));

		}
		
		return setPoseRotationInPlace(
			translateAtOrthogonalAngle(
				centeredPose,
				centeredPose.getRotation(),
				shiftAmount
			),
			centeredPose.getRotation()
		);
		
	}
	
}
