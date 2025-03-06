package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configuration.CANDevice;

public class Mailbox extends SubsystemBase {
	
	protected TalonFX motorController;
	
	public final Commands commands;
	
	public Mailbox() {
		
		this.motorController =
			new TalonFX(CANDevice.MAILBOX_MOTOR_CONTROLLER.id);
		this.commands = new Commands();
		
		this.motorController.getConfigurator().apply(Mailbox.getMotorConfig());
		
		
	}
	
	protected static MotorOutputConfigs getMotorConfig() {
		
		return (new MotorOutputConfigs())
			.withNeutralMode(NeutralModeValue.Brake)
			.withInverted(InvertedValue.CounterClockwise_Positive);
		
	}
	
	public void addToOrchestra(Orchestra orchestra) {
		
		orchestra.addInstrument(this.motorController);
		
	}
	
	public class Commands {
		
		protected Command spin(double speed) {
			
			return Mailbox.this.startEnd(
				() -> Mailbox.this.motorController.set(speed),
				Mailbox.this.motorController::stopMotor
			);
			
		}
		
		public Command acceptMail() {
			
			return this.spin(0.25);
			
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
	
}
