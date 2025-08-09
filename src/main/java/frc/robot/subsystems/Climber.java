package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.*;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.configuration.CANDevice;
import frc.robot.configuration.DIODevice;

import static edu.wpi.first.units.Units.*;

public class Climber extends SubsystemBase {
	
	protected static final Angle ENCODER_OFFSET = Degrees.of(269.164);
	
	protected static final Angle DEPLOYED_ANGLE = Degrees.of(-50);
	
	protected static final Angle CLIMBED_ANGLE = Degrees.of(40);
	
	protected static final double kP = 0.6;
	
	protected static final double kI = 0.0;
	
	protected static final double kD = 0.0;
	
	protected static final AngularVelocity DEFAULT_MAX_VELOCITY =
		DegreesPerSecond.of(60);
	
	protected static final AngularVelocity GENTLE_MAX_VELOCITY =
		DegreesPerSecond.of(5);
	
	protected static final AngularAcceleration MAX_ACCELERATION =
		DegreesPerSecondPerSecond.of(120);
	
	protected static final Angle ANGULAR_POSITION_TOLERANCE = Degrees.of(0.5);
	
	protected final TalonFX motorController;
	
	protected final DutyCycleEncoder encoder;
	
	protected boolean isEncoderReady;
	
	protected final ProfiledPIDController pidController;
	
	public final Commands commands;
	
	public final Triggers triggers;
	
	public Climber() {
		
		this.motorController = new TalonFX(CANDevice.CLIMBER_WINCH_MOTOR_CONTROLLER.id);
		this.encoder = new DutyCycleEncoder(DIODevice.CLIMBER_ENCODER.channel);
		this.isEncoderReady = false;
		this.pidController = new ProfiledPIDController(
			Climber.kP,
			Climber.kI,
			Climber.kD,
			new TrapezoidProfile.Constraints(
				Climber.DEFAULT_MAX_VELOCITY.in(DegreesPerSecond),
				Climber.MAX_ACCELERATION.in(DegreesPerSecondPerSecond)
			)
		);
		this.commands = new Commands();
		this.triggers = new Triggers();
		
		this.motorController.getConfigurator().apply(Climber.getMotorConfig());
		this.pidController.setTolerance(Climber.ANGULAR_POSITION_TOLERANCE.in(Degrees));
		this.pidController.setGoal(Climber.CLIMBED_ANGLE.in(Degrees));
		
	}
	
	protected static MotorOutputConfigs getMotorConfig() {
		
		return (new MotorOutputConfigs())
			.withNeutralMode(NeutralModeValue.Brake)
			.withInverted(InvertedValue.CounterClockwise_Positive);
		
	}
	
	protected void setMaxAngularVelocity(AngularVelocity velocity) {
		
		this.pidController.setConstraints(new TrapezoidProfile.Constraints(
			velocity.in(DegreesPerSecond),
			Climber.MAX_ACCELERATION.in(DegreesPerSecondPerSecond)
		));
		
	}
	
	public void addToOrchestra(Orchestra orchestra) {
		
		orchestra.addInstrument(this.motorController);
		
	}
	
	@Override
	public void periodic() {
		
//		double pidVoltage =
//			this.pidController.calculate(this.getAngle().in(Degrees));
//
//		this.motorController.setVoltage(MathUtil.clamp(pidVoltage, -13, 6));
		
		if (this.getAngle().lt(Degrees.of(-80))) return;
		
		this.motorController.setVoltage(
			this.pidController.calculate(this.getAngle().in(Degrees))
		);
		
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
			() -> this.pidController.getGoal().position,
			this.pidController::setGoal
		);
		
	}
	
	public class Commands {
		
		protected Command setMaxAngularVelocity(AngularVelocity velocity) {
			
			return Climber.this.runOnce(
				() -> Climber.this.setMaxAngularVelocity(velocity)
			);
			
		}
		
		public Command goToAngle(Angle angle) {
			
			return Climber.this.startEnd(
				() -> Climber.this.pidController.setGoal(angle.in(Degrees)),
				() -> Climber.this.pidController.setGoal(CLIMBED_ANGLE.in(Degrees))
			);
			
		}
		
		public Command deploy() {
			
			return this.setMaxAngularVelocity(DEFAULT_MAX_VELOCITY)
				.andThen(this.goToAngle(DEPLOYED_ANGLE));
			
		}
		
		public Command climb() {
			
			return this.setMaxAngularVelocity(DEFAULT_MAX_VELOCITY)
				.andThen(this.goToAngle(CLIMBED_ANGLE));
			
		}
		
		public Command unclimb() {
			
			return this.setMaxAngularVelocity(GENTLE_MAX_VELOCITY)
				.andThen(this.goToAngle(DEPLOYED_ANGLE))
				.finallyDo(() -> Climber.this.setMaxAngularVelocity(DEFAULT_MAX_VELOCITY));
			
		}
		
	}
	
	public class Triggers {
		
		public Trigger isAtAngle(Angle angle) {
			
			return new Trigger(() -> Climber.this.getAngle().isNear(angle, Degrees.of(2)));
			
		}
		
	}
	
}
