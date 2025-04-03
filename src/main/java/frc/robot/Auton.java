package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.configuration.FieldThird;
import frc.robot.configuration.ReefAlignment;
import frc.robot.configuration.ReefLevel;
import frc.robot.util.RobotPoseBuilder;

import java.util.function.Function;

import static edu.wpi.first.units.Units.*;

public enum Auton {
	
	NONE(
		"None (No Auton)",
		robot -> new WaitCommand(0)
	),
	
	CROSS_THE_LINE("Cross the Line", robot ->
		robot.swerve.commands.drive(() -> new Translation2d(-15, 0), () -> 0, true)
			.withTimeout(Seconds.of(4))
	),
	
	STANDARD_AUTON("Standard Auton", robot ->
//		robot.swerve.commands.drive(() -> new Translation2d(-15, 0), () -> 0, true)
//			.until(robot.odometry::hasVisionData)
		new InstantCommand()
			.andThen(robot.complexCommands.autoScoreOnReef(() -> robot.odometry.getFieldThird().getReefFrontAprilTagID(), ReefLevel.L4, ReefAlignment.RIGHT))
			.andThen(
				robot.swerve.commands.goToRelativePosition(() ->
					RobotPoseBuilder.fromPose(robot.odometry.getPose())
						.withFieldRelativeTranslation(new Translation2d(
							Inches.of(0),
							Inches.of(20).times(
								robot.odometry.getFieldThird() == FieldThird.LEFT
									? 1
									: -1
							)
						)).toPose(),
					InchesPerSecond.of(80),
					Inches.of(6),
					Degrees.of(5)
				).andThen(
					robot.swerve.commands.goToPosition(() ->
						RobotPoseBuilder.getCoralStationLoadingPose(
							robot.odometry.getFieldThird().getCoralStationAprilTagID()
						).toPose(),
						InchesPerSecond.of(80),
						Inches.of(0.25),
						Degrees.of(1),
						null
					).withDeadline(robot.swerve.commands.waitUntilAtPosition(
						() -> RobotPoseBuilder.getCoralStationLoadingPose(
							robot.odometry.getFieldThird().getCoralStationAprilTagID()
						).toPose(),
						Inches.of(0.5),
						Degrees.of(1)
					)).withTimeout(3)
				).andThen(new InstantCommand(() -> System.out.println("Got to position")))
				.andThen(
					robot.swerve.commands.drive(
						() -> new Translation2d(Inches.of(-10), Inches.of(0)),
						() -> 0,
						false
					).withDeadline(
						Commands.waitUntil(robot.intake.triggers.isCoralInUpperIntake())
							.andThen(
								robot.complexCommands.autoAcceptMail()
									.alongWith(robot.complexCommands.autofeedMailbox())
									.until(robot.intake.triggers.isCoralInLowerIntake())
							)
							.andThen(
								robot.complexCommands.autoAcceptMail()
									.alongWith(robot.complexCommands.autofeedMailbox())
									.until(robot.intake.triggers.isCoralInLowerIntake().negate())
							)
					)
				)
				.andThen(
					robot.complexCommands.autoScoreOnReef(ReefLevel.L4, ReefAlignment.RIGHT)
				)
					.onlyIf(() -> robot.odometry.getFieldThird() != FieldThird.CENTER)
			)
	),
	
	SINGLE_L1_CORAL_CENTER("Single L1 Coral (Center)", robot ->
		robot.swerve.commands.drive(() -> new Translation2d(-15, 0), () -> 0, true)
			.withTimeout(Seconds.of(4))
			.andThen(robot.mailbox.commands.feed().withTimeout(2))
	),
	
	SINGLE_L4_CORAL_CENTER("Single L4 Coral (Center)", robot ->
		new SequentialCommandGroup(
			robot.swerve.commands.drive(() -> new Translation2d(-15, 0), () -> 0, true).withTimeout(Seconds.of(6)),
			robot.swerve.commands.drive(() -> new Translation2d(10, 0), () -> 0, true).withTimeout(Seconds.of(0.25)),
			robot.swerve.commands.drive(() -> new Translation2d(0, 0), () -> 0, true).withTimeout(Seconds.of(0.1)),
			robot.complexCommands.scoreOnL4()
		)
	),
	
	SINGLE_CORAL("Single Coral", robot ->
		robot.swerve.commands.drive(() -> new Translation2d(-15, 0), () -> 0, true)
			.until(robot.odometry::hasVisionData)
			.andThen(robot.complexCommands.autoScoreOnReef(() -> robot.odometry.getFieldThird().getReefFrontAprilTagID(), ReefLevel.L4, ReefAlignment.LEFT))
	),
	
	DOUBLE_CORAL(
		"Double Coral",
		robot -> new WaitCommand(0)
	);
	
	/**
	 * The Shuffleboard widget used for selecting the auton to run.
	 */
	private static final SendableChooser<Auton> SHUFFLEBOARD_SELECTOR =
		new SendableChooser<>();
	
	/**
	 * The default Alliance color to assume if no alliance color is able to be
	 * fetched from the FMS.
	 */
	private static final Alliance DEFAULT_AUTON_ALLIANCE = Alliance.Red;
	
	/**
	 * The default auton to run if no auton is explicitly selected.
	 */
	private static final Auton DEFAULT_AUTON = Auton.SINGLE_CORAL;
	
	/**
	 * A flag indicating whether or not the Shuffleboard auton selector has been
	 * initialized.
	 */
	private static boolean hasShuffleboardSelectorBeenInitialized = false;
	
	/**
	 * The human-readable name of this auton.
	 */
	private final String humanReadableName;
	
	/**
	 * The function that supplies the Command object for this auton.
	 */
	private final Function<RobotContainer, Command> commandSupplier;
	
	/**
	 * Constructs a new Auton with the given name and command supplier.
	 *
	 * @param name The human-readable name of this auton.
	 * @param commandFunction The function that supplies the Command object for
	 * this auton.
	 */
	Auton(String name, Function<RobotContainer, Command> commandFunction) {
		
		this.humanReadableName = name;
		this.commandSupplier = commandFunction;
		
	}
	
	/**
	 * Initializes the Shuffleboard widget used for selecting the starting
	 * position of the robot.
	 */
	public static void initializeShuffleboardSelector() {
		
		if (Auton.hasShuffleboardSelectorBeenInitialized) return;
		else Auton.hasShuffleboardSelectorBeenInitialized = true;
		
		Auton.SHUFFLEBOARD_SELECTOR.setDefaultOption(
			Auton.DEFAULT_AUTON.getHumanReadableName(),
			Auton.DEFAULT_AUTON
		);
		
		for (Auton auton: Auton.values()) {
			
			if (auton == Auton.DEFAULT_AUTON) continue;
			
			Auton.SHUFFLEBOARD_SELECTOR.addOption(
				auton.getHumanReadableName(),
				auton
			);
			
		}
		
		Shuffleboard.getTab("Pre-match Tab").add(
			"Auton Chooser",
			Auton.SHUFFLEBOARD_SELECTOR
		);
		
	}
	
	/**
	 * Returns the auton selected by the driver from the Shuffleboard widget.
	 *
	 * @return The auton selected by the driver from the Shuffleboard widget.
	 */
	public static Auton getSelectedAuton() {
		
		return Auton.SHUFFLEBOARD_SELECTOR.getSelected();
		
	}
	
	/**
	 * Runs the auton currently selected by the Shuffleboard widget.
	 *
	 * @param robot The robot on which to run the auton.
	 */
	public static Command runSelectedAuton(RobotContainer robot) {
		
		Command autonCommand = Auton.getSelectedAuton().getCommand(robot);
		
		autonCommand.schedule();
		
		return autonCommand;
		
	}
	
	/**
	 * Returns the Command object for this auton.
	 *
	 * @param robot The robot on which to run the auton.
	 * @return The Command object for this auton.
	 */
	public Command getCommand(RobotContainer robot) {
		
		return this.commandSupplier.apply(robot);
		
	}
	
	/**
	 * Returns the human-readable name of this auton.
	 *
	 * @return The human-readable name of this auton.
	 */
	public String getHumanReadableName() {
		
		return this.humanReadableName;
		
	}
	
}
