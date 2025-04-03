package frc.robot.util;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configuration.FieldThird;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class RaptorsOdometry extends SubsystemBase {
	
	protected Swerve swerve;
	
	public Vision vision;
	
	protected SwerveDrivePoseEstimator poseEstimator;
	
	protected final Field2d field;
	
	protected boolean hasVisionData;
	
	protected AprilTag nearestReefAprilTag;
	
	public RaptorsOdometry() {
		
		this.swerve = null;
		this.vision = null;
		this.poseEstimator = null;
		this.field = new Field2d();
		this.hasVisionData = false;
		this.nearestReefAprilTag = null;
		
		SmartDashboard.putData("Field Odometry", this.field);
		
	}
	
	protected static Pose2d getInitialPose() {
		
		return RobotPoseBuilder.fromCenterFieldPose()
			.withFieldRelativeHeading(Rotation2d.k180deg)
			.toPose();
		
	}
	
	public void injectSwerve(Swerve swerve) {
		
		this.swerve = swerve;
		this.poseEstimator = new SwerveDrivePoseEstimator(
			swerve.getKinematics(),
			new Rotation2d(swerve.getFieldRelativeHeading()),
			swerve.getModulePositions(),
			RaptorsOdometry.getInitialPose()//,
//			VecBuilder.fill(1, 1, 0.7),
//			VecBuilder.fill(5, 5, 999999)
		);
		
	}
	
	public void injectVision(Vision vision) {
		
		this.vision = vision;
		
	}
	
	@Override
	public void periodic() {
		
		if (this.poseEstimator == null) return;
		
		this.integrateSwerveData();
		this.integrateVisionData();
		this.field.setRobotPose(this.getPose());
		
		AprilTag nearestReefAprilTag =
			VirtualField.getNearestReefAprilTag(this.getTranslation());
		
		if (nearestReefAprilTag != null) {
			
			this.nearestReefAprilTag = nearestReefAprilTag;
			this.field.getObject("nearest-april-tag")
				.setPose(nearestReefAprilTag.pose.toPose2d());
			
		}
		
	}
	
	public boolean hasVisionData() {
		
		return this.hasVisionData;
		
	}
	
	public Pose2d getPose() {
		
		return this.poseEstimator != null
			? this.poseEstimator.getEstimatedPosition()
			: RaptorsOdometry.getInitialPose();
		
	}
	
	public void setDisplaySetpoint(Pose2d setpoint) {
		
		this.field.getObject("setpoint").setPose(setpoint);
		
	}
	
	public void removeDisplaySetpoint() {
		
		this.field.getObject("setpoint").setPoses();
		
	}
	
	public Translation2d getTranslation() {
		
		return this.getPose().getTranslation();
		
	}
	
	public FieldThird getFieldThird() {
		
		return VirtualField.getFieldThirdForPosition(this.getTranslation());
		
	}
	
	public AprilTag getNearestReefAprilTag() {
		
		return this.nearestReefAprilTag;
		
	}
	
	public void integrateSwerveData() {
		
		this.poseEstimator.update(
			new Rotation2d(this.swerve.getFieldRelativeHeading()),
			this.swerve.getModulePositions()
		);
		
	}
	
	public void integrateVisionData() {
		
		for (Vision.VisionMeasurement measure: this.vision.getMeasurements()) {
			
			this.hasVisionData = true;
			
			this.poseEstimator.addVisionMeasurement(
				measure.visionRobotPoseMeters(),
				measure.timestampSeconds(),
				measure.visionMeasurementStdDevs()
			);
			
		}
	
	}
	
	public void resetPose(Pose2d pose) {
		
		if (this.poseEstimator != null) this.poseEstimator.resetPose(pose);
		
	}
	
}
