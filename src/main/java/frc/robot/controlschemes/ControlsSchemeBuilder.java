package frc.robot.controlschemes;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.configuration.ReefBranch;
import frc.robot.configuration.ReefLevel;
import frc.robot.configuration.ReefScoringMode;
import frc.robot.subsystems.Swerve;
import frc.robot.util.DoubleSupplierBuilder;
import frc.robot.util.ElevatorPosition;
import frc.robot.util.Point;
import frc.robot.util.PointSupplierBuilder;

import java.util.Map;

import static edu.wpi.first.units.Units.*;

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
	protected static final double LINEAR_INPUT_SMOOTHING_POWER = 3;
	
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
		
		Time timeToMaxVelocity = Seconds.of(0.125);
		LinearVelocity maxLinearVelocity = InchesPerSecond.of(300);
		AngularVelocity maxAngularVelocity = DegreesPerSecond.of(360);
		Swerve swerve = this.robot.swerve;
		
		swerve.setDefaultCommand(swerve.commands.drive(
			PointSupplierBuilder.fromLeftJoystick(controller)
				.normalizeXboxJoystickToNWU()
				.withClamp(-1, 1)
				.withScaledDeadband(JOYSTICK_DEADBAND)
				.withExponentialCurve(LINEAR_INPUT_SMOOTHING_POWER)
				.withScaling(maxLinearVelocity.in(InchesPerSecond))
				.withMaximumSlewRate(maxLinearVelocity.div(timeToMaxVelocity).in(InchesPerSecond.per(Second))),
			DoubleSupplierBuilder.fromRightX(controller)
				.withScaling(-1)
				.withClamp(-1, 1)
				.withScaledDeadband(JOYSTICK_DEADBAND)
				.withExponentialCurve(LINEAR_INPUT_SMOOTHING_POWER)
				.withScaling(maxAngularVelocity.in(DegreesPerSecond))
				.withMaximumSlewRate(maxAngularVelocity.div(timeToMaxVelocity).in(DegreesPerSecond.per(Second))),
			true
		));
		
		return this;
		
	}
	
	public ControlsSchemeBuilder useDPadForRobotRelativeDriving(CommandXboxController controller) {
		
		LinearVelocity speed = FeetPerSecond.of(1.5);
		
		controller.povUp().whileTrue(this.robot.swerve.commands.drive(() -> new Translation2d(speed.in(InchesPerSecond), 0), () -> 0, false));
		controller.povRight().whileTrue(this.robot.swerve.commands.drive(() -> new Translation2d(0, -speed.in(InchesPerSecond)), () -> 0, false));
		controller.povLeft().whileTrue(this.robot.swerve.commands.drive(() -> new Translation2d(0, speed.in(InchesPerSecond)), () -> 0, false));
		controller.povDown().whileTrue(this.robot.swerve.commands.drive(() -> new Translation2d(-speed.in(InchesPerSecond), 0), () -> 0, false));
		
		return this;
		
	}
	
	public ControlsSchemeBuilder useStartToResetFieldHeading(CommandXboxController controller) {
		
		controller.start().onTrue(
			this.robot.swerve.commands.calibrateFieldRelativeHeading()
		);
		
		return this;
		
	}
	
	public ControlsSchemeBuilder useStartButtonToCalibrateElevator(CommandXboxController controller) {
		
		controller.start().onTrue(
			this.robot.elevator.commands.calibrate()
		);
		
		return this;
		
	}
	
	public ControlsSchemeBuilder useABXYToScoreCoral(CommandXboxController controller) {
		
		controller.b().whileTrue(this.robot.complexCommands.scoreOnL1());
		
		controller.a().whileTrue(new SelectCommand<ReefScoringMode>(Map.ofEntries(
			Map.entry(ReefScoringMode.MANUAL, this.robot.complexCommands.scoreOnL2()),
			Map.entry(ReefScoringMode.LEFT, this.robot.complexCommands.autoScoreOnReef(ReefLevel.L2, ReefBranch.LEFT)),
			Map.entry(ReefScoringMode.RIGHT, this.robot.complexCommands.autoScoreOnReef(ReefLevel.L2, ReefBranch.RIGHT))
		), () -> ReefScoringMode.ACTIVE));
		
		controller.x().whileTrue(new SelectCommand<ReefScoringMode>(Map.ofEntries(
			Map.entry(ReefScoringMode.MANUAL, this.robot.complexCommands.scoreOnL3()),
			Map.entry(ReefScoringMode.LEFT, this.robot.complexCommands.autoScoreOnReef(ReefLevel.L3, ReefBranch.LEFT)),
			Map.entry(ReefScoringMode.RIGHT, this.robot.complexCommands.autoScoreOnReef(ReefLevel.L3, ReefBranch.RIGHT))
		), () -> ReefScoringMode.ACTIVE));
		
		controller.y().whileTrue(new SelectCommand<ReefScoringMode>(Map.ofEntries(
			Map.entry(ReefScoringMode.MANUAL, this.robot.complexCommands.scoreOnL4()),
			Map.entry(ReefScoringMode.LEFT, this.robot.complexCommands.autoScoreOnReef(ReefLevel.L4, ReefBranch.LEFT)),
			Map.entry(ReefScoringMode.RIGHT, this.robot.complexCommands.autoScoreOnReef(ReefLevel.L4, ReefBranch.RIGHT))
		), () -> ReefScoringMode.ACTIVE));
		
		return this;
		
	}
	
	public ControlsSchemeBuilder usePOVButtonsToSwitchReefScoringModes(
		CommandXboxController controller
	) {
		
		controller.povDown().onTrue(new InstantCommand(() -> ReefScoringMode.setMode(ReefScoringMode.MANUAL)));
		controller.povLeft().onTrue(new InstantCommand(() -> ReefScoringMode.setMode(ReefScoringMode.LEFT)));
		controller.povRight().onTrue(new InstantCommand(() -> ReefScoringMode.setMode(ReefScoringMode.RIGHT)));
		
		return this;
		
	}
	
	public ControlsSchemeBuilder useTriggersToRemoveAlgae(CommandXboxController controller) {
		
		controller.leftTrigger(TRIGGER_THRESHOLD).whileTrue(
			this.robot.complexCommands.removeAlgaeAtLevel(ElevatorPosition.L2_ALGAE_REMOVAL)
		);
		
		controller.rightTrigger(TRIGGER_THRESHOLD).whileTrue(
			this.robot.complexCommands.removeAlgaeAtLevel(ElevatorPosition.L3_ALGAE_REMOVAL)
		);
		
		return this;
		
	}
	
	public ControlsSchemeBuilder useBumpersToClimb(CommandXboxController controller) {
		
		controller.leftBumper().and(controller.rightBumper())
			.whileTrue(this.robot.climber.commands.deploy());
		
		return this;
		
	}
	
	public ControlsSchemeBuilder useYToClimb(CommandXboxController controller) {
		
		controller.y().whileTrue(
			this.robot.climber.commands.deploy()
//				.alongWith(this.robot.lights.commands.flashingWhite())
		);
		
		return this;
		
	}
	
	public ControlsSchemeBuilder useBackButtonToUnclimb(CommandXboxController controller) {
		
		controller.back().whileTrue(
			this.robot.climber.commands.unclimb()
		);
		
		return this;
		
	}
	
	public ControlsSchemeBuilder usePovUpToDistanceL4Score(CommandXboxController controller) {
		
		controller.povUp().whileTrue(
			this.robot.complexCommands.scoreOnL4Distant()
		);
		
		return this;
		
	}
	
}
