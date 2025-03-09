package frc.robot.util;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class AprilTagHelper {
	
	protected static AprilTagHelper instance;
	
	protected final AprilTagFieldLayout aprilTags;
	
	protected AprilTagHelper() {
		
		this.aprilTags = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
		
	}
	
	public static AprilTagHelper getInstance() {
		
		if (AprilTagHelper.instance == null) {
			
			AprilTagHelper.instance = new AprilTagHelper();
			
		}
		
		return AprilTagHelper.instance;
		
	}
	
	public static AprilTag getAprilTag(int id) {
		
		return AprilTagHelper.getInstance().aprilTags.getTags().get(id - 1);
		
	}
	
}
