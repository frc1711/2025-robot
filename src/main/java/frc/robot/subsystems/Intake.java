package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.configuration.CANDevice;
import frc.robot.devicewrappers.RaptorsLaserCAN;

import static edu.wpi.first.units.Units.Inches;

public class Intake extends SubsystemBase {
	
	protected final TalonFX motorController;
	
	protected final RaptorsLaserCAN upperBeamBreak;
	
	protected final RaptorsLaserCAN lowerBeamBreak;
	
	public final Commands commands;
	
	public final Triggers triggers;
	
	public Intake() {
		
		this.motorController =
			new TalonFX(CANDevice.AGITATOR_MOTOR_CONTROLLER.id);
		this.upperBeamBreak = new RaptorsLaserCAN(CANDevice.INTAKE_UPPER_LASER_CAN);
		this.lowerBeamBreak = new RaptorsLaserCAN(CANDevice.INTAKE_LOWER_LASER_CAN);
		this.commands = new Commands();
		this.triggers = new Triggers();
		
		this.motorController.getConfigurator().apply(Intake.getMotorConfig());
		
	}
	
	protected static MotorOutputConfigs getMotorConfig() {
		
		return (new MotorOutputConfigs())
			.withNeutralMode(NeutralModeValue.Brake)
			.withInverted(InvertedValue.Clockwise_Positive);
		
	}
	
	public void addToOrchestra(Orchestra orchestra) {
		
		orchestra.addInstrument(this.motorController);
		
	}
	
	@Override
	public void initSendable(SendableBuilder builder) {
		
		builder.addDoubleProperty(
			"Upper Beambreak Distance (Inches)",
			() -> {
				Distance distance = this.upperBeamBreak.getDistance();
				
				if (distance == null) return 0;
				else return distance.in(Inches);
				
			},
			null
		);
		
		builder.addDoubleProperty(
			"Lower Beambreak Distance (Inches)",
			() -> {
				
				Distance distance = this.lowerBeamBreak.getDistance();
				
				if (distance == null) return 0;
				return distance.in(Inches);
				
			},
			null
		);
		
	}
	
	public class Commands {
		
		public Command feed(double speed) {
			
			return Intake.this.startEnd(
				() -> Intake.this.motorController.set(speed),
				Intake.this.motorController::stopMotor
			);
			
		}
		
		public Command feed() {
			
			return this.feed(1);
			
		}
		
		public Command unfeed(double speed) {
			
			return this.feed(-speed);
			
		}
		
		public Command unfeed() {
			
			return this.unfeed(1);
			
		}

		public Command waitUntilCoralIsInUpperIntake() {

			return edu.wpi.first.wpilibj2.command.Commands.waitUntil(
				Intake.this.triggers.isCoralInUpperIntake()
			);

		}
		
		public Command waitUntilCoralIsNotInUpperIntake() {
			
			return edu.wpi.first.wpilibj2.command.Commands.waitUntil(
				Intake.this.triggers.isCoralInUpperIntake().negate()
			);
			
		}

		public Command waitUntilCoralIsInLowerIntake() {

			return edu.wpi.first.wpilibj2.command.Commands.waitUntil(
				Intake.this.triggers.isCoralInLowerIntake()
			);

		}

		public Command waitUntilCoralIsNotInLowerIntake() {

			return edu.wpi.first.wpilibj2.command.Commands.waitUntil(
				Intake.this.triggers.isCoralInLowerIntake().negate()
			);

		}
		
	}
	
	public class Triggers {
		
		public Trigger isCoralInUpperIntake() {
			
			return Intake.this.upperBeamBreak.getDistanceTrigger(Inches.of(8));
			
		}
		
		public Trigger isCoralInLowerIntake() {
			
			return Intake.this.lowerBeamBreak.getDistanceTrigger(Inches.of(1.5));
			
		}
		
		public Trigger isCoralInIntake() {
			
			return this.isCoralInUpperIntake().or(this.isCoralInLowerIntake());
			
		}
		
	}
	
}
