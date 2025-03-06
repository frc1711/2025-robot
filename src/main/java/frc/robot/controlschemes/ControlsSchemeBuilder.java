package frc.robot.controlschemes;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.util.DoubleSupplierBuilder;
import frc.robot.util.PointSupplierBuilder;

public class ControlsSchemeBuilder {
	
	/**
	 * The threshold at which the triggers of the controller should be
	 * considered to be pressed.
	 */
	protected static final double TRIGGER_THRESHOLD = 0.5;
	
	/**
	 * The deadband to apply to the joysticks of the controller.
	 */
	protected static final double JOYSTICK_DEADBAND = 0.1;
	
	/**
	 * The power to raise the input of the joysticks to for smoothing.
	 */
	protected static final double LINEAR_INPUT_SMOOTHING_POWER = 2;
	
	protected final RobotContainer robot;
	
	public ControlsSchemeBuilder(RobotContainer robot) {
		
		this.robot = robot;
		
	}
	
	public ControlsSchemeBuilder configureDefaultRobotCommands() {
		
		this.robot.intake.setDefaultCommand(this.robot.complexCommands.autofeedMailbox());
		this.robot.mailbox.setDefaultCommand(this.robot.complexCommands.autoAcceptMail());
		
		return this;
		
	}
	
	public ControlsSchemeBuilder useControllerJoysticksForDriving(
		CommandXboxController controller
	) {
		
		this.robot.swerve.setDefaultCommand(
			this.robot.swerve.commands.driveFieldRelative(
				PointSupplierBuilder.fromLeftJoystick(controller)
					.normalizeXboxJoystickToNWU()
					.withClamp(-1, 1)
					.withScaledDeadband(JOYSTICK_DEADBAND)
					.withExponentialCurve(LINEAR_INPUT_SMOOTHING_POWER),
				DoubleSupplierBuilder.fromRightX(controller)
					.withScaling(-1)
					.withClamp(-1, 1)
					.withScaledDeadband(JOYSTICK_DEADBAND)
					.withExponentialCurve(LINEAR_INPUT_SMOOTHING_POWER)
					.withScaling(2)
			)
		);
		
		return this;
		
	}
	
	public ControlsSchemeBuilder useABXYToScoreCoral(CommandXboxController controller) {
		
		controller.b().whileTrue(this.robot.complexCommands.scoreOnL1());
		controller.a().whileTrue(this.robot.complexCommands.scoreOnL2());
		controller.x().whileTrue(this.robot.complexCommands.scoreOnL3());
		controller.y().whileTrue(this.robot.complexCommands.scoreOnL4());
		
		return this;
		
	}
	
	public ControlsSchemeBuilder useBumpersToClimb(CommandXboxController controller) {
		
		controller.leftBumper().and(controller.rightBumper())
			.whileTrue(this.robot.climber.commands.deploy().alongWith(
				this.robot.lights.commands.redBlueFlash()
			));
		
		return this;
		
	}
	
}
