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

import java.util.List;
import java.util.stream.Stream;

import static edu.wpi.first.units.Units.*;

public class VirtualField {
	
	public static final Distance FIELD_LENGTH = Meters.of(17.5482504);
	
	public static final Distance FIELD_WIDTH = Meters.of(8.0519016);
	
	public static final AprilTagFieldLayout APRIL_TAGS =
		AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
	
//	protected static final
	
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
		
		return VirtualField.getReefAprilTags(
			DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)
		);
		
	}
	
	public static Stream<AprilTag> getCoralStationAprilTags(DriverStation.Alliance alliance) {
		
		return (alliance == DriverStation.Alliance.Blue
			? Stream.of(12, 13)
			: Stream.of(1, 2)
		).map(VirtualField::getAprilTagByID);
		
	}
	
	public static Stream<AprilTag> getCoralStationAprilTags() {
		
		return VirtualField.getCoralStationAprilTags(
			DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)
		);
		
	}
	
	public static Translation2d getReefCenterPoint(DriverStation.Alliance alliance) {
		
		List<Translation2d> reefAprilTagPoints =
			VirtualField.getReefAprilTags(alliance)
				.map(tag -> tag.pose.getTranslation().toTranslation2d())
				.toList();
		
		return reefAprilTagPoints.stream()
			.reduce((point, acc) -> new Translation2d(
				point.getX() + acc.getX(),
				point.getY() + acc.getY()
			)).get().div(reefAprilTagPoints.size());
		
	}
	
	public static Translation2d getReefCenterPoint() {
		
		return VirtualField.getReefCenterPoint(
			DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)
		);
		
	}
	
	public static AprilTag getNearestReefAprilTag(
		DriverStation.Alliance alliance,
		Translation2d position
	) {
		
		Translation2d reefCenterPoint = VirtualField.getReefCenterPoint(alliance);
		Translation2d positionRelativeToReef = position.minus(reefCenterPoint);
		Rotation2d rotationAroundReef = new Rotation2d(positionRelativeToReef.getX(), positionRelativeToReef.getY());
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
			DriverStation.getAlliance().orElse(DriverStation.Alliance.Red),
			position
		);
		
	}
	
	public static FieldThird getFieldThirdForPosition(
		DriverStation.Alliance alliance,
		Translation2d position
	) {
		
		Distance reefSectionWidth = Inches.of(37);
		Distance leftSideCutoff = FIELD_WIDTH.minus(reefSectionWidth).div(2);
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
			DriverStation.getAlliance().orElse(DriverStation.Alliance.Red),
			position
		);
		
	}
	
}
