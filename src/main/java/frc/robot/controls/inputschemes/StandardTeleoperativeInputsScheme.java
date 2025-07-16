package frc.robot.controls.inputschemes;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.controls.InputScheme;
import frc.robot.controls.InputSchemeBuilder;

public class StandardTeleoperativeInputsScheme implements InputScheme {
	
	@Override
	public void configureControllerInputs(
		RobotContainer robotContainer,
		CommandXboxController controller1,
		CommandXboxController controller2
	) {
		
		(new InputSchemeBuilder(robotContainer))
			.configureDefaultRobotCommands()
			.useControllerJoysticksForDriving(controller1)
			.useRBButtonForSlowMode(controller1)
			.useYButtonForCoastMode(controller1)
			.useTriggersToLoad(controller1)
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
		
		InputScheme.super.periodic(robot, controller1, controller2);
		
	}
	
	@Override
	public void exit(
		RobotContainer robot,
		CommandXboxController controller1,
		CommandXboxController controller2
	) {
		
		InputScheme.super.exit(robot, controller1, controller2);
		
	}
	
}
