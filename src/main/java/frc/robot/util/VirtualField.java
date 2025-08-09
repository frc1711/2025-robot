package frc.robot.util;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.configuration.FieldThird;
import frc.robot.math.Point;

import java.util.List;
import java.util.stream.Stream;

import static edu.wpi.first.units.Units.*;

public class VirtualField {
	
	public static final Distance FIELD_LENGTH = Meters.of(17.5482504);
	
	public static final Distance FIELD_WIDTH = Meters.of(8.0519016);
	
	public static final Distance REEF_SECTION_WIDTH = Inches.of(37);
	
	public static final AprilTagFieldLayout APRIL_TAGS =
		AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
	
	public static final DriverStation.Alliance DEFAULT_ALLIANCE =
		DriverStation.Alliance.Red;
	
	public static DriverStation.Alliance getAlliance() {
		
		return DriverStation.getAlliance()
			.orElse(VirtualField.DEFAULT_ALLIANCE);
		
	}

	public static boolean isRedAlliance() {

		return DriverStation.Alliance.Red.equals(VirtualField.getAlliance());

	}

	public static boolean isBlueAlliance() {

		return DriverStation.Alliance.Blue.equals(VirtualField.getAlliance());

	}
	
	public static AprilTag getAprilTagByID(int id) {
		
		return VirtualField.APRIL_TAGS.getTags().get(id - 1);
		
	}
	
	public static Stream<AprilTag> getReefAprilTags(DriverStation.Alliance alliance) {
		
		return (alliance == DriverStation.Alliance.Blue
			? Stream.of(17, 18, 19, 20, 21, 22)
			: Stream.of(6, 7, 8, 9, 10, 11)
		).map(VirtualField::getAprilTagByID);
		
	}
	
	public static Stream<AprilTag> getReefAprilTags() {
		
		return VirtualField.getReefAprilTags(VirtualField.getAlliance());
		
	}
	
	public static Stream<AprilTag> getCoralStationAprilTags(DriverStation.Alliance alliance) {
		
		return (alliance == DriverStation.Alliance.Blue
			? Stream.of(12, 13)
			: Stream.of(1, 2)
		).map(VirtualField::getAprilTagByID);
		
	}
	
	public static Stream<AprilTag> getCoralStationAprilTags() {
		
		return VirtualField.getCoralStationAprilTags(VirtualField.getAlliance());
		
	}
	
	public static Point getReefCenterPoint(DriverStation.Alliance alliance) {
		
		return Point.average(
			VirtualField.getReefAprilTags(alliance)
				.map(tag -> tag.pose.getTranslation().toTranslation2d())
				.toArray(Translation2d[]::new)
		);
		
	}
	
	public static Translation2d getReefCenterPoint() {
		
		return VirtualField.getReefCenterPoint(VirtualField.getAlliance());
		
	}
	
	public static AprilTag getNearestReefAprilTag(
		DriverStation.Alliance alliance,
		Translation2d position
	) {
		
		Translation2d reefCenterPoint = VirtualField.getReefCenterPoint(alliance);
		Translation2d positionRelativeToReef = position.minus(reefCenterPoint);
		double positionRelativeToReefX = positionRelativeToReef.getX();
		double positionRelativeToReefY = positionRelativeToReef.getY();
		Rotation2d rotationAroundReef =
			positionRelativeToReefX == 0 && positionRelativeToReefY == 0
				? Rotation2d.kZero
				: new Rotation2d(positionRelativeToReefX, positionRelativeToReefY);
		List<AprilTag> reefAprilTags = VirtualField.getReefAprilTags(alliance).toList();
		
		AprilTag nearestTag = reefAprilTags.get(0);
		double nearestTagDistance = 360;
		
		for (AprilTag reefAprilTag: reefAprilTags) {
			
			double angularDistance = Math.abs(
				reefAprilTag.pose.getRotation().toRotation2d()
					.minus(rotationAroundReef)
					.getDegrees()
			);
			
			if (angularDistance < nearestTagDistance) {
				
				nearestTag = reefAprilTag;
				nearestTagDistance = angularDistance;
				
			}
			
		}
		
		return nearestTag;
		
	}
	
	public static AprilTag getNearestReefAprilTag(Translation2d position) {
		
		return VirtualField.getNearestReefAprilTag(
			VirtualField.getAlliance(),
			position
		);
		
	}
	
	public static FieldThird getFieldThirdForPosition(
		DriverStation.Alliance alliance,
		Translation2d position
	) {
		
		Distance leftSideCutoff = FIELD_WIDTH.minus(REEF_SECTION_WIDTH).div(2);
		Distance rightSideCutoff = FIELD_WIDTH.minus(leftSideCutoff);
		Distance robotYPosition = position.getMeasureY();
		
		if (robotYPosition.lt(leftSideCutoff)) {
			
			return alliance == DriverStation.Alliance.Red
				? FieldThird.LEFT
				: FieldThird.RIGHT;
			
		} else if (robotYPosition.gt(rightSideCutoff)) {
			
			return alliance == DriverStation.Alliance.Red
				? FieldThird.RIGHT
				: FieldThird.LEFT;
			
		} else return FieldThird.CENTER;
		
	}
	
	public static FieldThird getFieldThirdForPosition(Translation2d position) {
		
		return VirtualField.getFieldThirdForPosition(
			VirtualField.getAlliance(),
			position
		);
		
	}
	
}
