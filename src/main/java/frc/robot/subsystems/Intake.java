package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.configuration.CANDevice;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;

public class Intake extends SubsystemBase {
	
	protected final TalonFX motorController;
	
	protected final LaserCan upperBeamBreak;
	
	protected final LaserCan lowerBeamBreak;
	
	public final Commands commands;
	
	public final Triggers triggers;
	
	public Intake() {
		
		this.motorController =
			new TalonFX(CANDevice.AGITATOR_MOTOR_CONTROLLER.id);
		this.upperBeamBreak = new LaserCan(CANDevice.INTAKE_UPPER_LASER_CAN.id);
		this.lowerBeamBreak = new LaserCan(CANDevice.INTAKE_LOWER_LASER_CAN.id);
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
			() -> Millimeters.of(this.upperBeamBreak.getMeasurement().distance_mm).in(Inches),
			null
		);
		
		builder.addDoubleProperty(
			"Lower Beambreak Distance (Inches)",
			() -> Millimeters.of(this.lowerBeamBreak.getMeasurement().distance_mm).in(Inches),
			null
		);
		
	}
	
	public class Commands {
		
		protected Command spin(double speed) {
			
			return Intake.this.startEnd(
				() -> Intake.this.motorController.set(speed),
				Intake.this.motorController::stopMotor
			);
			
		}
		
		public Command feed(double speed) {
			
			return this.spin(speed);
			
		}
		
		public Command feed() {
			
			return this.feed(1);
			
		}
		
		public Command unfeed() {
			
			return this.spin(-1);
			
		}
		
	}
	
	public class Triggers {
		
		public Trigger isCoralInUpperIntake() {
			
			return new Trigger(() -> {
				
				Distance measuredDistance = Millimeters.of(
					Intake.this.upperBeamBreak.getMeasurement().distance_mm
				);
				
				return measuredDistance.lte(Inches.of(8));
				
			});
			
		}
		
		public Trigger isCoralInLowerIntake() {
			
			return new Trigger(() -> {
				
				Distance measuredDistance = Millimeters.of(
					Intake.this.lowerBeamBreak.getMeasurement().distance_mm
				);
				
				return measuredDistance.lte(Inches.of(1.5));
				
			});
			
		}
		
	}
	
}
