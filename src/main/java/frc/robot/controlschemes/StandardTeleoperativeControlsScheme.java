package frc.robot.controlschemes;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;

public class StandardTeleoperativeControlsScheme implements ControlsScheme {
	
	@Override
	public void configureControls(
		RobotContainer robotContainer,
		CommandXboxController controller1,
		CommandXboxController controller2
	) {
		
		(new ControlsSchemeBuilder(robotContainer))
			.configureDefaultRobotCommands()
			.useControllerJoysticksForDriving(controller1)
			.useDPadForRobotRelativeDriving(controller1)
			.useStartToResetFieldHeading(controller1)
			.useABXYToScoreCoral(controller2)
			.usePOVButtonsToSwitchReefScoringModes(controller2)
			.useTriggersToRemoveAlgae(controller2)
			.useBumpersToClimb(controller2)
			.useBackButtonToUnclimb(controller2)
			.useStartButtonToCalibrateElevator(controller2);
		
	}
	
	@Override
	public void periodic(
		RobotContainer robot,
		CommandXboxController controller1,
		CommandXboxController controller2
	) {
		
		ControlsScheme.super.periodic(robot, controller1, controller2);
		
	}
	
	@Override
	public void exit(
		RobotContainer robot,
		CommandXboxController controller1,
		CommandXboxController controller2
	) {
		
		ControlsScheme.super.exit(robot, controller1, controller2);
		
	}
	
}
