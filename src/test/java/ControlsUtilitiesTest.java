import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.configuration.ReefAlignment;
import frc.robot.util.RobotPoseBuilder;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.stream.Stream;

import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.*;

public class ControlsUtilitiesTest {
	
	static AprilTagFieldLayout aprilTags =
		AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
	
	private static void assertPoseEquals(Pose2d expected, Pose2d actual) {
		
		Translation2d expectedTranslation = expected.getTranslation();
		Translation2d actualTranslation = actual.getTranslation();
		double distance = expectedTranslation.getDistance(actualTranslation);
		
		assertTrue(
			Math.abs(distance) < 0.001,
			"Pose translation differs from expected (expected: " + expectedTranslation + ", actual: " + actualTranslation + ", distance: " + distance + ")"
		);
		
		double expectedRotation = expected.getRotation().getDegrees();
		double actualRotation = actual.getRotation().getDegrees();
		double offset = expectedRotation - actualRotation;
		
		assertTrue(
			Math.abs(offset) < 0.001,
			"Pose rotation differs from expected (expected: " + expectedRotation + ", actual: " + actualRotation + ", offset: " + offset + ")"
		);
		
	}

	private static Stream<Arguments> provideAprilTagTestCases() {
		
		return Stream.of(
			Arguments.of( 6, new Pose2d(Meters.of(13.474446 + 0.2286), Meters.of(3.306318 - 0.3959), Rotation2d.fromDegrees(120))),
			Arguments.of( 7, new Pose2d(Meters.of(13.890498 + 0.4572), Meters.of(4.0259), Rotation2d.fromDegrees(180))),
			Arguments.of( 8, new Pose2d(Meters.of(13.474446 + 0.2286), Meters.of(4.745482 + 0.3959), Rotation2d.fromDegrees(-120))),
			Arguments.of( 9, new Pose2d(Meters.of(12.643358 - 0.2286), Meters.of(4.745482 + 0.3959), Rotation2d.fromDegrees(-60))),
			Arguments.of(10, new Pose2d(Meters.of(12.227306 - 0.4572), Meters.of(4.0259), Rotation2d.fromDegrees(0))),
			Arguments.of(11, new Pose2d(Meters.of(12.643358 - 0.2286), Meters.of(3.306318 - 0.3959), Rotation2d.fromDegrees(60))),
			Arguments.of(17, new Pose2d(Meters.of(4.073906 - 0.2286), Meters.of(3.306318 - 0.3959), Rotation2d.fromDegrees(60))),
			Arguments.of(18, new Pose2d(Meters.of(4.073906 - 0.2286), Meters.of(4.0259), Rotation2d.fromDegrees(0))),
			Arguments.of(21, new Pose2d(Meters.of(5.321046 + 0.4572), Meters.of(4.0259), Rotation2d.fromDegrees(180)))
		);
		
	}
	
	@ParameterizedTest
	@MethodSource("provideAprilTagTestCases")
	public void doesPoseOffsetWork(int tagID, Pose2d desiredPose) {
		
		Pose2d tagPose = aprilTags.getTagPose(tagID).get().toPose2d();
		
		System.out.println("Tag " + tagID + " Pose: (" + tagPose.getX() + ", " + tagPose.getY() + ")");
		
//		Pose2d robotPose = ControlsUtilities.getRobotPoseForReefAprilTagPose(tagPose);
//		Pose2d robotPose = RobotPoseHelper.getCenteredRobotPoseForReefAprilTag(tagID, ReefLevel.L2);
		Pose2d robotPose = RobotPoseBuilder.getReefScoringPose(tagID, ReefAlignment.CENTER).toPose();
		
		assertPoseEquals(robotPose, desiredPose);
		
	}

}
