package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.configuration.StatusLightsPattern;

import static edu.wpi.first.units.Units.Seconds;

public class StatusLights extends SubsystemBase {
	
	protected final PWM blinken;
	
	protected StatusLightsPattern currentPattern;
	
	public final Commands commands;
	
	public StatusLights() {
		
		this.blinken = new PWM(0);
		this.commands = new Commands();
		
	}
	
	public void set(StatusLightsPattern pattern) {
		
		this.currentPattern = pattern;
		this.blinken.setPulseTimeMicroseconds(this.currentPattern.pulseWidth);
		
	}
	
	public class Commands {
		
		protected Command set(StatusLightsPattern pattern) {
			
			return StatusLights.this.runOnce(
				() -> StatusLights.this.set(pattern)
			);
			
		}
		
		public Command raptorsGreen() {
			
			return this.set(StatusLightsPattern.SOLID_COLORS_DARK_GREEN);
			
		}
		
		public Command redBlueFlash() {
			
			return this.set(StatusLightsPattern.SOLID_COLORS_RED)
				.andThen(edu.wpi.first.wpilibj2.command.Commands.waitTime(Seconds.of(0.5)))
				.andThen(this.set(StatusLightsPattern.SOLID_COLORS_BLUE))
				.andThen(edu.wpi.first.wpilibj2.command.Commands.waitTime(Seconds.of(0.5)))
				.repeatedly();
			
		}
		
		public Command flashingWhite() {
			
			return this.set(StatusLightsPattern.FIXED_PALETTE_PATTERN_STROBE_WHITE);
			
		}
		
		public Command goToNextPattern() {
			
			return StatusLights.this.runOnce(() -> {
				
				StatusLightsPattern[] patterns = StatusLightsPattern.values();
				int previousID = StatusLights.this.currentPattern.patternID;
				int previousIndex = previousID - 1;
				int nextIndex = previousIndex + 1;
				
				if (nextIndex >= patterns.length) nextIndex = 0;
				
				StatusLights.this.set(patterns[nextIndex]);
				
			});
			
		}
		
		public Command cycleThroughAll() {
			
			return this.goToNextPattern()
				.andThen(new WaitCommand(Seconds.of(1)))
				.repeatedly();
			
		}
		
	}
	
}
