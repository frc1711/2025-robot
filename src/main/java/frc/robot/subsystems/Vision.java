package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;
import frc.robot.devicewrappers.LimelightHelpers;
import frc.robot.util.VirtualField;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class Vision {
	
	protected static final String[] LIMELIGHTS = new String[] {
		"limelight-left", // IP: 10.17.11.12
		"limelight-right" // IP: 10.17.11.11
	};
	
	protected static final int[] APRIL_TAG_IDS_TO_USE = {
		// Red Coral Stations
		1, 2,
		// Red Reef Faces
		6, 7, 8, 9, 10, 11,
		// Blue Coral Stations
		12, 13,
		// Blue Reef Faces
		17, 18, 19, 20, 21, 22
	};
	
	protected final Supplier<Angle> headingSupplier;
	
	protected final Supplier<LinearVelocity> linearVelocitySupplier;
	
	protected final Supplier<AngularVelocity> angularVelocitySupplier;
	
	protected int[] currentAprilTagFilter;
	
	public Vision(
		Supplier<Angle> headingSupplier,
		Supplier<LinearVelocity> linearVelocitySupplier,
		Supplier<AngularVelocity> angularVelocitySupplier
	) {
		
		this.headingSupplier = headingSupplier;
		this.linearVelocitySupplier = linearVelocitySupplier;
		this.angularVelocitySupplier = angularVelocitySupplier;
		this.currentAprilTagFilter = null;
	
	}
	
	public void setAprilTagFilter(int[] tagIDs) {
	
		this.currentAprilTagFilter = tagIDs;
	
	}
	
	public void resetAprilTagFilter() {
		
		this.currentAprilTagFilter = null;
		
	}
	
	public List<VisionMeasurement> getMeasurements() {
		
		boolean shouldUpdateVision = (
			this.angularVelocitySupplier.get().lt(DegreesPerSecond.of(80))
		);
		
		if (!shouldUpdateVision) return List.of();
		
		double limelightYaw = this.headingSupplier.get().in(Degrees) +
			(VirtualField.isRedAlliance() ? 180 : 0);
		
		int[] aprilTagFilter = this.currentAprilTagFilter != null
			? this.currentAprilTagFilter
			: Vision.APRIL_TAG_IDS_TO_USE;
		
		for (String limelightName: Vision.LIMELIGHTS) {
			
//			LimelightHelpers.SetFiducialIDFiltersOverride(
//				limelightName,
//				aprilTagFilter
//			);
			
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
		
		double maxTagDistInMeters = Feet.of(10).in(Meters);
		double maxTagAmbiguity = 0.6;
		LinearVelocity linearVelocity = this.linearVelocitySupplier.get();
		AngularVelocity angularVelocity = this.angularVelocitySupplier.get();
		
		return Arrays.stream(Vision.LIMELIGHTS)
			.map(LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2)
			.filter(estimate -> (
				estimate != null &&
				estimate.pose != null &&
				estimate.tagCount > 0 &&
				estimate.avgTagDist < maxTagDistInMeters &&
				estimate.rawFiducials[0].ambiguity < maxTagAmbiguity
			))
			.map(estimate -> {
				
				double ambiguityScaling = estimate.rawFiducials[0].ambiguity + 1;
				Time kLatency = Milliseconds.of(estimate.latency).times(3);
//				Distance baselineLinearDeviation = Inches.of(Math.pow(2, 2 * ambiguityScaling));
				Distance baselineLinearDeviation = Inches.of(4).times(ambiguityScaling);
				Distance possibleRobotMovement = linearVelocity.times(kLatency);
//				Distance tagDisplacementAmbiguity = Meters.of(estimate.avgTagDist)
//					.times(ambiguityScaling);
				Distance linearDeviation = baselineLinearDeviation
					.plus(possibleRobotMovement);
//					.plus(tagDisplacementAmbiguity);
				double linearDeviationMeters = linearDeviation.in(Meters);
				Angle baselineAngularDeviation = Degrees.of(30);
				Angle possibleRobotRotation = angularVelocity.times(kLatency);
				Angle angularDeviation = baselineAngularDeviation.plus(possibleRobotRotation);
				double angularDeviationDegrees = angularDeviation.in(Degrees);
				
				return new VisionMeasurement(
					estimate.pose,
					estimate.timestampSeconds,
					VecBuilder.fill(
						linearDeviationMeters,
						linearDeviationMeters,
						angularDeviationDegrees
					)
				);
				
			})
			.toList();
		
	}
	
	public record VisionMeasurement(
		Pose2d visionRobotPoseMeters,
		double timestampSeconds,
		Matrix<N3, N1> visionMeasurementStdDevs
	) {}
	
}
