package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.configuration.ReefAlignment;
import frc.robot.configuration.ReefLevel;
import frc.robot.configuration.StatusLightsPattern;
import frc.robot.util.*;

import java.util.function.IntSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class ComplexCommands {
	
	protected final RobotContainer robot;
	
	public ComplexCommands(RobotContainer robot) {
		
		this.robot = robot;
		
	}

	public Command drive() {

		return this.drive(this.robot.controller1);

	}

	public Command drive(CommandXboxController controller) {

		return this.robot.swerve.commands.drive(
			ChassisSpeedsSupplierBuilder.fromControllerJoysticks(controller)
				.withFieldRelative(this.robot.swerve)
				.withAdditional(ChassisSpeedsSupplierBuilder.fromControllerDPad(controller))
				.withSlowModeCheck(this.robot.swerve)
				.withMaxVelocityCheck()
				.withMaxAccelerationCheck()
		).finallyDo(this.robot.swerve::stop);

	}
	
	public Command autofeedMailbox() {
		
		Trigger canAcceptMail = this.robot.elevator.triggers
			.isAtPosition(ElevatorPosition.L2_SCORING)
			.and(this.robot.intake.triggers.isCoralInIntake());
		
		return Commands.waitUntil(canAcceptMail)
			.andThen(this.robot.intake.commands.feed())
			.onlyWhile(canAcceptMail);
	
	}
	
	public Command autoAcceptMail() {
		
		Trigger canAcceptMail = this.robot.elevator.triggers
			.isAtPosition(ElevatorPosition.RESTING)
				.and(this.robot.intake.triggers.isCoralInLowerIntake());
		
		return Commands.waitUntil(canAcceptMail)
			.andThen(this.robot.mailbox.commands.feed(0.25))
			.onlyWhile(canAcceptMail);

	}
	
	public Command raiseAndShoot(ReefLevel level) {

		Command moveElevatorToPosition = this.robot.elevator.commands.goTo(level.elevatorScoringPosition);
		Command waitForElevatorToMoveToPosition = this.robot.elevator.commands.waitUntilAtPosition(level.elevatorScoringPosition);
		Command shoot = this.robot.mailbox.commands.feed().withTimeout(Seconds.of(0.5));
		Command raiseAndShoot = this.drive().alongWith(
			moveElevatorToPosition.withDeadline(waitForElevatorToMoveToPosition.andThen(shoot))
		);
		Runnable resetLights = () -> this.robot.lights.set(StatusLightsPattern.SOLID_COLORS_WHITE);
		
		if (level.equals(ReefLevel.L4)) {
			
			raiseAndShoot = raiseAndShoot.andThen(
				this.robot.elevator.commands.goTo(ElevatorPosition.L4_TIP)
					.alongWith(this.robot.mailbox.commands.feed())
					.until(this.robot.elevator.triggers.isAtPosition(ElevatorPosition.L4_TIP))
			);
			
		}
		
		return raiseAndShoot.finallyDo(resetLights);
		
	}
	
	public Command unloadMailbox() {
		
		return this.robot.mailbox.commands.unfeed()
			.alongWith(this.robot.intake.commands.feed(-0.5));
		
	}
	
	public Command loadMailbox(double speed) {
		
		return this.robot.mailbox.commands.feed(speed)
			.alongWith(this.robot.intake.commands.feed(speed));
		
	}
	
	public Command loadMailbox() {
		
		return this.robot.mailbox.commands.feed()
			.alongWith(this.robot.intake.commands.feed());
		
	}
	
	public Command autoScoreOnReef(IntSupplier reefTagID, ReefLevel level, ReefAlignment branch) {
		
		ElevatorPosition elevatorPosition = switch (level) {
			case L1_TROUGH -> ElevatorPosition.L1_SCORING;
			case L2 -> ElevatorPosition.L2_SCORING;
			case L3 -> ElevatorPosition.L3_SCORING;
			case L4 -> ElevatorPosition.L4_SCORING;
		};
		Distance extra = level.equals(ReefLevel.L4)
				? Inches.of(-2)
				: Inches.of(0);
		Command startLog = new LogCommand(
			"Attempting to auto-score on reef. [April tag ID: " +
			reefTagID.getAsInt() + ", level: " + level + ", branch: " + branch +
			"]"
		);
		Supplier<Pose2d> poseSupplier = () ->
			RobotPoseBuilder.getReefScoringPose(reefTagID.getAsInt(), branch)
				.withRobotRelativeTranslation(new Translation2d(extra, Inches.of(0)))
				.get();
		Command goToScoringPosition = this.robot.swerve.commands.goToPosition(
			poseSupplier,
			InchesPerSecond.of(40),
			Inches.of(0.25),
			Degrees.of(1),
			() -> new int[] { reefTagID.getAsInt() }
		);
		Command waitUntilPositionedToScore = this.robot.swerve.commands
			.waitUntilAtPosition(poseSupplier, Inches.of(0.375), Degrees.of(1))
			.alongWith(Commands.waitUntil(this.robot.elevator.triggers.isAtPosition(elevatorPosition)));
		Command raiseElevator = new InstantCommand(() -> this.robot.elevator.goToPosition(elevatorPosition));
		Command scoreCoral = this.robot.mailbox.commands.feed(level).withTimeout(0.6);
		Command positionCoral = robot.complexCommands.autoAcceptMail()
			.alongWith(robot.complexCommands.autofeedMailbox())
			.until(robot.intake.triggers.isCoralInUpperIntake().negate().and(robot.intake.triggers.isCoralInLowerIntake().negate()));

		if (level == ReefLevel.L4) {
			
			scoreCoral = scoreCoral.andThen(
				this.robot.elevator.commands.goTo(ElevatorPosition.L4_TIP)
					.alongWith(this.robot.mailbox.commands.feed())
					.until(this.robot.elevator.triggers.isAtPosition(ElevatorPosition.L4_TIP))
			);
			
		}
		
		return startLog.andThen(
			positionCoral
		.andThen(new WaitCommand(0.25))
//		.andThen(goToCalibrationPosition)
		.andThen(goToScoringPosition
			.alongWith(raiseElevator)
			.alongWith(autoAcceptMail())
			.withDeadline(waitUntilPositionedToScore)
			.andThen(scoreCoral)
		)).finallyDo(() -> this.robot.elevator.goToPosition(ElevatorPosition.RESTING));
		
	}
	
	public Command autoScoreOnReef(ReefLevel level, ReefAlignment branch) {
		
		return this.autoScoreOnReef(
			() -> this.robot.odometry.getNearestReefAprilTag().ID,
			level,
			branch
		).onlyIf(() -> this.robot.odometry.getNearestReefAprilTag() != null);
		
	}
	
	public Command removeAlgaeAtLevel(ElevatorPosition position) {
		
		return this.robot.elevator.commands.goTo(position)
			.alongWith(this.robot.mailbox.commands.feed(0.25).withTimeout(0.25))
			.finallyDo(() -> this.robot.elevator.goToPosition(ElevatorPosition.RESTING));
		
	}
	
}
