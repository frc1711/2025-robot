package frc.robot.controlschemes;

import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.configuration.ReefAlignment;
import frc.robot.configuration.ReefLevel;
import frc.robot.configuration.ReefAlignmentMode;
import frc.robot.util.*;

import java.util.Map;

import static edu.wpi.first.units.Units.*;

public class ControlsSchemeBuilder {
	
	/**
	 * The threshold at which the triggers of the controller should be
	 * considered to be pressed.
	 */
	protected static final double TRIGGER_THRESHOLD = 0.5;
	
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
		
		this.robot.swerve.setDefaultCommand(this.robot.complexCommands.drive(controller));
		
		return this;
		
	}
	
	public ControlsSchemeBuilder useRBButtonForSlowMode(
		CommandXboxController controller
	) {
		
		controller.rightBumper().onTrue(new InstantCommand(() -> this.robot.swerve.isSlowModeEnabled = true));
		controller.rightBumper().onFalse(new InstantCommand(() -> this.robot.swerve.isSlowModeEnabled = false));
		
		return this;
		
	}
	
	public ControlsSchemeBuilder useYButtonForCoastMode(
		CommandXboxController controller
	) {
		
		controller.y().whileTrue(
			new InstantCommand(() -> this.robot.swerve.setDriveMotorIdleState(SparkBaseConfig.IdleMode.kCoast))
				.andThen(this.robot.swerve.commands.xMode(InchesPerSecond.of(-15)))
		);
		
		controller.y().onFalse(
			new InstantCommand(() -> this.robot.swerve.setDriveMotorIdleState(SparkBaseConfig.IdleMode.kBrake))
		);
		
		return this;
		
	}
	
	public ControlsSchemeBuilder useDPadForRobotRelativeDriving(CommandXboxController controller) {
		
		LinearVelocity speed = FeetPerSecond.of(1.5);
		
		controller.povUp().whileTrue(this.robot.swerve.commands.drive(() -> new Translation2d(speed.in(MetersPerSecond), 0), () -> 0, false));
		controller.povDown().whileTrue(this.robot.swerve.commands.drive(() -> new Translation2d(-speed.in(MetersPerSecond), 0), () -> 0, false));
		
		controller.povLeft().whileTrue(this.robot.swerve.commands.drive(() -> new Translation2d(0, speed.in(MetersPerSecond)), () -> 0, false));
		controller.povRight().whileTrue(this.robot.swerve.commands.drive(() -> new Translation2d(0, -speed.in(MetersPerSecond)), () -> 0, false));
		
		return this;
		
	}
	
	public ControlsSchemeBuilder useStartToResetFieldHeading(CommandXboxController controller) {
		
		controller.start().onTrue(
			this.robot.swerve.commands.calibrateFieldRelativeHeading()
		);
		
		return this;
		
	}
	
	public ControlsSchemeBuilder useTriggersToLoad(CommandXboxController controller) {
		
		Command goToNearestCoralStation = this.robot.swerve.commands.goToPosition(() ->
			RobotPoseBuilder.getCoralStationLoadingPose(
				robot.odometry.getFieldThird().getCoralStationAprilTagID()
			).toPose(),
			InchesPerSecond.of(80),
			Inches.of(0.5),
			Degrees.of(2),
			null
		);
		
		controller.leftTrigger(TRIGGER_THRESHOLD)
			.and(controller.rightTrigger(TRIGGER_THRESHOLD))
			.whileTrue(goToNearestCoralStation);
		
		return this;
		
	}
	
	public ControlsSchemeBuilder useStartButtonToCalibrateElevator(CommandXboxController controller) {
		
		controller.start().onTrue(this.robot.elevator.commands.calibrate());
		
		return this;
		
	}
	
	public ControlsSchemeBuilder useABXYToScoreCoral(CommandXboxController controller) {
		
		controller.b().whileTrue(this.robot.complexCommands.scoreOnL1());
		
		controller.a().whileTrue(new SelectCommand<ReefAlignmentMode>(Map.ofEntries(
			Map.entry(ReefAlignmentMode.MANUAL, this.robot.complexCommands.scoreOnL2()),
			Map.entry(ReefAlignmentMode.LEFT, this.robot.complexCommands.autoScoreOnReef(ReefLevel.L2, ReefAlignment.LEFT)),
			Map.entry(ReefAlignmentMode.RIGHT, this.robot.complexCommands.autoScoreOnReef(ReefLevel.L2, ReefAlignment.RIGHT))
		), () -> ReefAlignmentMode.ACTIVE));
		
		controller.x().whileTrue(new SelectCommand<ReefAlignmentMode>(Map.ofEntries(
			Map.entry(ReefAlignmentMode.MANUAL, this.robot.complexCommands.scoreOnL3()),
			Map.entry(ReefAlignmentMode.LEFT, this.robot.complexCommands.autoScoreOnReef(ReefLevel.L3, ReefAlignment.LEFT)),
			Map.entry(ReefAlignmentMode.RIGHT, this.robot.complexCommands.autoScoreOnReef(ReefLevel.L3, ReefAlignment.RIGHT))
		), () -> ReefAlignmentMode.ACTIVE));
		
		controller.y().whileTrue(new SelectCommand<ReefAlignmentMode>(Map.ofEntries(
			Map.entry(ReefAlignmentMode.MANUAL, this.robot.complexCommands.scoreOnL4()),
			Map.entry(ReefAlignmentMode.LEFT, this.robot.complexCommands.autoScoreOnReef(ReefLevel.L4, ReefAlignment.LEFT)),
			Map.entry(ReefAlignmentMode.RIGHT, this.robot.complexCommands.autoScoreOnReef(ReefLevel.L4, ReefAlignment.RIGHT))
		), () -> ReefAlignmentMode.ACTIVE));
		
		return this;
		
	}
	
	public ControlsSchemeBuilder usePOVButtonsToSwitchReefScoringModes(
		CommandXboxController controller
	) {
		
		controller.povDown().onTrue(new InstantCommand(() -> ReefAlignmentMode.setMode(ReefAlignmentMode.MANUAL)));
		controller.povLeft().onTrue(new InstantCommand(() -> ReefAlignmentMode.setMode(ReefAlignmentMode.LEFT)));
		controller.povRight().onTrue(new InstantCommand(() -> ReefAlignmentMode.setMode(ReefAlignmentMode.RIGHT)));
		
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
