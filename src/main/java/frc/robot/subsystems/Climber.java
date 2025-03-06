package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.configuration.CANDevice;

import static edu.wpi.first.units.Units.*;

public class Climber extends SubsystemBase {
	
	protected static final Angle ENCODER_OFFSET = Degrees.of(269.164);
	
	protected static final Angle DEPLOYED_ANGLE = Degrees.of(-50);
	
	protected static final Angle CLIMBED_ANGLE = Degrees.of(40);
	
	protected static final double kP = 0.4;
	
	protected static final double kI = 0.0;
	
	protected static final double kD = 0.0;
	
	protected static final Angle ANGULAR_POSITION_TOLERANCE = Degrees.of(0.5);
	
	protected final TalonFX motorController;
	
	protected final DutyCycleEncoder encoder;
	
	protected final PIDController pidController;
	
	public final Commands commands;
	
	public Climber() {
		
		this.motorController = new TalonFX(CANDevice.CLIMBER_WINCH_MOTOR_CONTROLLER.id);
		this.encoder = new DutyCycleEncoder(2);
		this.pidController = new PIDController(Climber.kP, Climber.kI, Climber.kD);
		this.commands = new Commands();
		
		this.motorController.getConfigurator().apply(Climber.getMotorConfig());
		this.pidController.setTolerance(Climber.ANGULAR_POSITION_TOLERANCE.in(Degrees));
		this.pidController.setSetpoint(Climber.CLIMBED_ANGLE.in(Degrees));
		
	}
	
	protected static MotorOutputConfigs getMotorConfig() {
		
		return (new MotorOutputConfigs())
			.withNeutralMode(NeutralModeValue.Brake)
			.withInverted(InvertedValue.CounterClockwise_Positive);
		
	}
	
	public void addToOrchestra(Orchestra orchestra) {
		
		orchestra.addInstrument(this.motorController);
		
	}
	
	@Override
	public void periodic() {
		
		this.motorController.setVoltage(MathUtil.clamp(
			this.pidController.calculate(this.getAngle().in(Degrees)),
			-13,
			6
		));
		
	}
	
	public Angle getAngle() {
		
		return Rotations.of(this.encoder.get())
			.minus(ENCODER_OFFSET);
		
	}
	
	@Override
	public void initSendable(SendableBuilder builder) {
		
		builder.addDoubleProperty(
			"Arm Angle (Degrees)",
			() -> this.getAngle().in(Degrees),
			null
		);
		
		builder.addDoubleProperty(
			"Arm Angle Setpoint (Degrees)",
			this.pidController::getSetpoint,
			this.pidController::setSetpoint
		);
		
	}
	
	public class Commands {
		
		public Command deploy() {
			
			return new StartEndCommand(
				() -> Climber.this.pidController.setSetpoint(Climber.DEPLOYED_ANGLE.in(Degrees)),
				() -> Climber.this.pidController.setSetpoint(Climber.CLIMBED_ANGLE.in(Degrees))
			);
			
		}
		
	}
	
}
