package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.configuration.CANDevice;
import frc.robot.util.ElevatorPosition;

import static edu.wpi.first.units.Units.*;

public class Elevator extends SubsystemBase {
	
	protected final TalonFX motorController;
	
	protected final MotionMagicVoltage request;
	
	protected final VoltageOut sysIdControl = new VoltageOut(0);
	
	public final Commands commands;
	
	public final Triggers triggers;
	
	public Elevator() {
		
		this.motorController = new TalonFX(CANDevice.ELEVATOR_MOTOR_CONTROLLER.id);
		this.request = new MotionMagicVoltage(0);
		this.commands = new Commands();
		this.triggers = new Triggers();
		
		this.motorController.getConfigurator().apply(Elevator.getMotorConfig());
		this.request.Slot = 0;
		
		SmartDashboard.putData(this.commands.setCurrentPositionAsZero());
		SmartDashboard.putData(this.commands.calibrate());
		
	}
	
	protected static TalonFXConfiguration getMotorConfig() {
		
		TalonFXConfiguration config = new TalonFXConfiguration();
		
		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		
		config.Slot0.kS = 0.086574;
		config.Slot0.kV = 0.027473;
		config.Slot0.kA = 0;
		config.Slot0.kG = 0.099146;
		
		config.Slot0.kP = 8.8814;
		config.Slot0.kI = 0;
		config.Slot0.kD = 0.011978;
		
		config.MotionMagic.MotionMagicCruiseVelocity = 100;
		config.MotionMagic.MotionMagicAcceleration = 200;
		config.MotionMagic.MotionMagicJerk = 2800;
		
		config.HardwareLimitSwitch.ForwardLimitEnable = false;
		config.HardwareLimitSwitch.ReverseLimitEnable = false;
		
		return config;
		
	}
	
	public void addToOrchestra(Orchestra orchestra) {
		
		orchestra.addInstrument(this.motorController);
		
	}
	
	public ElevatorPosition getPosition() {
		
		return ElevatorPosition.fromMotorShaftAngle(
			this.motorController.getPosition().getValue()
		);
		
	}
	
	public void goToPosition(ElevatorPosition position) {
		
		this.motorController.setControl(
			this.request.withPosition(position.getMotorShaftAngle())
		);
		
	}
	
	/**
	 * Sets the current position of the elevator to represent the zeroed state
	 * (i.e. when the elevator is all the way down/fully stowed).
	 */
	protected void setCurrentPositionAsZero() {
		
		this.motorController.setPosition(0);
		
	}
	
	@Override
	public void initSendable(SendableBuilder builder) {
		
		builder.addDoubleProperty(
			"Stage 1 Travel Offset (Inches)",
			() -> this.getPosition().getStage1TravelOffset().in(Inches),
			null
		);
		
		builder.addDoubleProperty(
			"Stage 2 Travel Offset (Inches)",
			() -> this.getPosition().getStage2TravelOffset().in(Inches),
			null
		);
		
		builder.addDoubleProperty(
			"Lower Mailbox Height (Inches)",
			() -> this.getPosition().getLowerMailboxHeight().in(Inches),
			null
		);
		
		builder.addDoubleProperty(
			"Upper Mailbox Height (Inches)",
			() -> this.getPosition().getUpperMailboxHeight().in(Inches),
			null
		);
		
		builder.addDoubleProperty(
			"Elevator Shaft Angle (Rotations)",
			() -> this.getPosition().getOutputShaftAngle().in(Rotations),
			null
		);
		
		builder.addDoubleProperty(
			"Motor Shaft Angle (Rotations)",
			() -> this.getPosition().getMotorShaftAngle().in(Rotations),
			null
		);
		
		builder.addDoubleProperty(
			"Motor Torque Current (Amps)",
			() -> this.motorController.getTorqueCurrent().getValue().in(Amps),
			null
		);
		
	}
	
	public class Commands {
		
		public Command simpleMove(double speed) {
			
			return Elevator.this.startEnd(
				() -> Elevator.this.motorController.set(speed),
				Elevator.this.motorController::stopMotor
			);
			
		}
		
		public Command goTo(ElevatorPosition position) {
			
			return Elevator.this.startEnd(
				() -> Elevator.this.goToPosition(position),
				() -> Elevator.this.goToPosition(ElevatorPosition.RESTING)
			);
			
		}
		
		public Command waitUntilAtPosition(
			ElevatorPosition position,
			Distance tolerance
		) {
			
			return edu.wpi.first.wpilibj2.command.Commands
				.waitUntil(Elevator.this.triggers.isAtPosition(position, tolerance));
			
		}
		
		public Command waitUntilAtPosition(ElevatorPosition position) {

			return edu.wpi.first.wpilibj2.command.Commands
				.waitUntil(Elevator.this.triggers.isAtPosition(position));
			
		}
		
		public Command setCurrentPositionAsZero() {
			
			return edu.wpi.first.wpilibj2.command.Commands.runOnce(
				Elevator.this::setCurrentPositionAsZero
			).withName("Set Current Position as Zero")
			.ignoringDisable(true);
			
		}
		
		public Command calibrate() {
			
			return new FunctionalCommand(
				() -> {
					System.out.println("First calibration movement...");
					Elevator.this.motorController.set(-0.1);
				},
				() -> {},
				(Boolean wasInterrupted) -> {
					System.out.println("Finished first calibration movement.");
					Elevator.this.motorController.stopMotor();
					Elevator.this.setCurrentPositionAsZero();
				},
				() -> Elevator.this.motorController.getTorqueCurrent().getValue().lt(Amps.of(-20)),
				Elevator.this
			)
				.andThen(this.simpleMove(0.1).withTimeout(0.5))
				.andThen(() -> Elevator.this.motorController.set(-0.02))
				.andThen(new WaitCommand(0.5))
				.andThen(new FunctionalCommand(
					() -> {
						System.out.println("Second calibration movement...");
					},
					() -> {},
					(Boolean wasInterrupted) -> {
						System.out.println("Finished second calibration movement.");
						Elevator.this.motorController.stopMotor();
						Elevator.this.setCurrentPositionAsZero();
					},
					() -> Elevator.this.motorController.getTorqueCurrent().getValue().lt(Amps.of(-10)),
					Elevator.this
				))
				.andThen(this.goTo(ElevatorPosition.RESTING))
				.withName("Calibrate");
			
		}
		
	}
	
	public class Triggers {
		
		public Trigger isAtPosition(ElevatorPosition position, Distance tolerance) {

			return new Trigger(() ->
				Elevator.this.getPosition().getStage2TravelOffset()
					.isNear(position.getStage2TravelOffset(), tolerance)
			);
			
		}
		
		public Trigger isAtPosition(ElevatorPosition position) {
			
			return this.isAtPosition(position, Inches.of(0.125));
			
		}
		
	}
	
}
