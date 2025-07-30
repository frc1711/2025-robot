package frc.robot.configuration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.util.ElevatorPosition;

import java.util.function.Function;

import static edu.wpi.first.units.Units.Inches;

public enum ReefLevel {
	
	L1_TROUGH(robot -> {

		Command waitUntilAtUnloadingHeight = robot.elevator.commands
			.waitUntilAtPosition(ElevatorPosition.RESTING, Inches.of(0.125));
		
		Trigger coralIsInLowerIntake = robot.intake.triggers.isCoralInLowerIntake();

		Command loadLowerMailbox = robot.elevator.commands.goTo(ElevatorPosition.L1_LOADING)
			.alongWith(
				robot.elevator.commands.waitUntilAtPosition(ElevatorPosition.L1_LOADING)
					.andThen(robot.complexCommands.loadMailbox().until(coralIsInLowerIntake.negate()))
				);

		return waitUntilAtUnloadingHeight
			.andThen(robot.complexCommands.unloadMailbox().until(coralIsInLowerIntake))
			.andThen(robot.complexCommands.unloadMailbox().until(coralIsInLowerIntake.negate()))
			.andThen(robot.complexCommands.loadMailbox(0.2).until(coralIsInLowerIntake))
			.andThen(loadLowerMailbox.until(coralIsInLowerIntake.negate()))
			.andThen(robot.complexCommands.simpleScore(ElevatorPosition.L1_SCORING, 1, 1));
		
	}),
	
	L2(robot -> robot.complexCommands.simpleScore(ElevatorPosition.L2_SCORING, 0.75, 0.5)),
	
	L3(robot -> robot.complexCommands.simpleScore(ElevatorPosition.L3_SCORING, 1, 0.5)),
	
	L4(robot ->
		robot.complexCommands.simpleScore(ElevatorPosition.L4_SCORING, 0.5, 0.5)
			.andThen(
				robot.elevator.commands.goTo(ElevatorPosition.L4_TIP)
					.alongWith(robot.mailbox.commands.feed())
					.until(robot.elevator.triggers.isAtPosition(ElevatorPosition.L4_TIP))
			)
	);

	final Function<RobotContainer, Command> scoringCommand;

	ReefLevel(Function<RobotContainer, Command> scoringCommand) {

		this.scoringCommand = scoringCommand;

	}
	
	public Command score(RobotContainer robot) {
		
		return this.scoringCommand.apply(robot);
		
	}
	
}
