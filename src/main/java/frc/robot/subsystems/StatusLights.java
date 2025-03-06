package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configuration.StatusLightsPattern;

import static edu.wpi.first.units.Units.Seconds;

public class StatusLights extends SubsystemBase {
	
	protected final PWM blinken;
	
	public final Commands commands;
	
	public StatusLights() {
		
		this.blinken = new PWM(0);
		this.commands = new Commands();
		
		this.setDefaultCommand(this.commands.raptorsGreen());
		
	}
	
	public void set(StatusLightsPattern pattern) {
		
		this.blinken.setPulseTimeMicroseconds(pattern.pulseWidth);
		
	}
	
	public class Commands {
		
		protected Command set(StatusLightsPattern pattern) {
			
			return StatusLights.this.runOnce(
				() -> StatusLights.this.set(pattern)
			);
			
		}
		
		public Command raptorsGreen() {
			
			return this.set(StatusLightsPattern.SOLID_COLORS_LAWN_GREEN);
			
		}
		
		public Command redBlueFlash() {
			
			return this.set(StatusLightsPattern.SOLID_COLORS_RED)
				.andThen(edu.wpi.first.wpilibj2.command.Commands.waitTime(Seconds.of(0.5)))
				.andThen(this.set(StatusLightsPattern.SOLID_COLORS_BLUE))
				.andThen(edu.wpi.first.wpilibj2.command.Commands.waitTime(Seconds.of(0.5)))
				.repeatedly();
			
		}
		
	}
	
}
