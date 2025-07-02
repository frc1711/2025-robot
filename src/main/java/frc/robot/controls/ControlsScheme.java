package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;

public interface ControlsScheme {
	
	/**
	 * Configures the controls for this control scheme.
	 */
	void configureControls(
		RobotContainer robotContainer,
		CommandXboxController controller1,
		CommandXboxController controller2
	);
	
	default void periodic(
		RobotContainer robot,
		CommandXboxController controller1,
		CommandXboxController controller2
	) {}
	
	default void exit(
		RobotContainer robot,
		CommandXboxController controller1,
		CommandXboxController controller2
	) {}
	
}
