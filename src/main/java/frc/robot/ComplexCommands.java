package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.ElevatorPosition;

public class ComplexCommands {
	
	protected final RobotContainer robot;
	
	public ComplexCommands(RobotContainer robot) {
		
		this.robot = robot;
		
	}
	
	public Command autofeedMailbox() {
		
		Trigger mailboxIsInPlace = this.robot.elevator.triggers
			.isAtPosition(ElevatorPosition.L2_SCORING);
		Trigger coralInIntake = this.robot.intake.triggers.isCoralInUpperIntake()
			.or(this.robot.intake.triggers.isCoralInLowerIntake());
	
		return Commands.waitUntil(mailboxIsInPlace.and(coralInIntake))
			.andThen(this.robot.intake.commands.feed())
			.onlyWhile(mailboxIsInPlace.and(coralInIntake));
	
	}
	
	public Command autoAcceptMail() {
		
		Trigger mailboxIsInPlace = this.robot.elevator.triggers
			.isAtPosition(ElevatorPosition.L2_SCORING);
		Trigger coralInIntake = this.robot.intake.triggers.isCoralInLowerIntake();
		
		return Commands.waitUntil(mailboxIsInPlace.and(coralInIntake))
			.andThen(this.robot.mailbox.commands.acceptMail())
			.onlyWhile(mailboxIsInPlace.and(coralInIntake));
		
	}
	
	protected Command simpleScore(ElevatorPosition position) {
		
		Command moveElevatorToPosition =
			this.robot.elevator.commands.goTo(position);
		Command waitForElevatorToMoveToPosition =
			Commands.waitUntil(this.robot.elevator.triggers.isAtPosition(position));
			this.robot.elevator.commands.waitToReachHeight(position);
		Command shoot = this.robot.mailbox.commands.feed().withTimeout(0.5);
		
		return moveElevatorToPosition
			.withDeadline(waitForElevatorToMoveToPosition.andThen(shoot));
		
	}
	
	public Command unloadMailbox() {
		
		return this.robot.mailbox.commands.unfeed()
			.alongWith(this.robot.intake.commands.unfeed());
		
	}
	
	public Command loadMailbox(double speed) {
		
		return this.robot.mailbox.commands.feed(speed)
			.alongWith(this.robot.intake.commands.feed(speed));
		
	}
	
	public Command loadMailbox() {
		
		return this.robot.mailbox.commands.feed()
			.alongWith(this.robot.intake.commands.feed());
		
	}
	
	public Command scoreOnL1() {
		
		Command waitUntilAtUnloadingHeight =
			this.robot.elevator.commands.waitToReachHeight(ElevatorPosition.RESTING);
		
		Command loadLowerMailbox = this.robot.elevator.commands.goTo(ElevatorPosition.L1_LOADING)
			.alongWith(
				this.robot.elevator.commands.waitToReachHeight(ElevatorPosition.L1_LOADING)
					.andThen(this.loadMailbox().until(this.robot.intake.triggers.isCoralInLowerIntake().negate()))
				
			);
		
		return waitUntilAtUnloadingHeight
			.andThen(this.unloadMailbox().until(this.robot.intake.triggers.isCoralInLowerIntake()))
			.andThen(this.unloadMailbox().until(this.robot.intake.triggers.isCoralInLowerIntake().negate()))
			.andThen(this.loadMailbox(0.2).until(this.robot.intake.triggers.isCoralInLowerIntake()))
			.andThen(loadLowerMailbox.until(this.robot.intake.triggers.isCoralInLowerIntake().negate()))
			.andThen(this.simpleScore(ElevatorPosition.L1_SCORING));
		
	}
	
	public Command scoreOnL2() {
		
		return this.simpleScore(ElevatorPosition.L2_SCORING);
		
	}
	
	public Command scoreOnL3() {
		
		return this.simpleScore(ElevatorPosition.L3_SCORING);
		
	}
	
	public Command scoreOnL4() {
		
		return this.simpleScore(ElevatorPosition.L4_SCORING).andThen(
			this.robot.elevator.commands.goTo(ElevatorPosition.L4_TIP)
				.alongWith(this.robot.mailbox.commands.feed())
				.until(this.robot.elevator.triggers.isAtPosition(ElevatorPosition.L4_TIP))
		);
		
	}
	
}
