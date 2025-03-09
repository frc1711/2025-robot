package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.configuration.ReefBranch;
import frc.robot.configuration.ReefLevel;
import frc.robot.util.Point;

import java.util.function.Function;

import static edu.wpi.first.units.Units.Seconds;

public enum Auton {
	
	NONE(
		"None (No Auton)",
		robot -> new WaitCommand(0)
	),
	
	CROSS_THE_LINE("Cross the Line", robot ->
		robot.swerve.commands.drive(() -> new Point(-15, 0), () -> 0, true)
			.withTimeout(Seconds.of(4))
	),
	
	SINGLE_L1_CORAL_CENTER("Single L1 Coral (Center)", robot ->
		robot.swerve.commands.drive(() -> new Point(-15, 0), () -> 0, true)
			.withTimeout(Seconds.of(4))
			.andThen(robot.mailbox.commands.feed().withTimeout(2))
	),
	
	SINGLE_L4_CORAL_CENTER("Single L4 Coral (Center)", robot ->
		new SequentialCommandGroup(
			robot.swerve.commands.drive(() -> new Point(-15, 0), () -> 0, true).withTimeout(Seconds.of(6)),
			robot.swerve.commands.drive(() -> new Point(10, 0), () -> 0, true).withTimeout(Seconds.of(0.25)),
			robot.swerve.commands.drive(() -> new Point(0, 0), () -> 0, true).withTimeout(Seconds.of(0.1)),
			robot.complexCommands.scoreOnL4()
		)
	),
	
	SINGLE_CORAL("Single Coral", robot ->
		robot.swerve.commands.drive(() -> new Point(-15, 0), () -> 0, true)
			.until(robot.swerve::hasReefAprilTagLock)
			.andThen(robot.complexCommands.autoScoreOnReef(ReefLevel.L3, ReefBranch.LEFT))
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
