package frc.robot.util;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.configuration.RobotDimensions;

import static edu.wpi.first.units.Units.*;

/**
 * A collection of reusable utility methods for common controls-related
 * operations.
 */
public class ControlsUtilities {
    
    // Prevent this class from being instantiated.
	private ControlsUtilities() {}
	


	public static Translation2d applyMaxNorm(Translation2d translation, double maxNorm) {

		return translation.times(Math.min(maxNorm/translation.getNorm(), 1));

	}
    
}
