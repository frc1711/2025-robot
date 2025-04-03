package frc.robot.controlschemes;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;

public class TestingTeleoperativeControlsScheme implements ControlsScheme {
	
	@Override
	public void configureControls(
		RobotContainer robotContainer,
		CommandXboxController controller1,
		CommandXboxController controller2
	) {
		
		(new ControlsSchemeBuilder(robotContainer))
			.configureDefaultRobotCommands()
			.useControllerJoysticksForDriving(controller1)
			.useTriggersToLoad(controller1)
			.useABXYToScoreCoral(controller1)
			.usePOVButtonsToSwitchReefScoringModes(controller1)
			.useBumpersToClimb(controller1)
			.useBackButtonToUnclimb(controller1)
			.useStartToResetFieldHeading(controller1);
		
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
