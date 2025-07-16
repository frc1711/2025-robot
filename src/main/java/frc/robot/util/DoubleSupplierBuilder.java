package frc.robot.util;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.math.DoubleUtilities;

import java.util.function.DoubleSupplier;

public class DoubleSupplierBuilder implements DoubleSupplier {
	
	protected final DoubleSupplier supplier;
	
	public DoubleSupplierBuilder(DoubleSupplier initial) {
		
		this.supplier = initial;
		
	}
	
	public static DoubleSupplierBuilder fromLeftX(XboxController controller) {
		
		return new DoubleSupplierBuilder(controller::getLeftX);
		
	}
	
	public static DoubleSupplierBuilder fromLeftX(
		CommandXboxController controller
	) {
		
		return new DoubleSupplierBuilder(controller::getLeftX);
		
	}
	
	public static DoubleSupplierBuilder fromLeftY(XboxController controller) {
		
		return new DoubleSupplierBuilder(controller::getLeftY);
		
	}
	
	public static DoubleSupplierBuilder fromLeftY(
		CommandXboxController controller
	) {
		
		return new DoubleSupplierBuilder(controller::getLeftY);
		
	}
	
	public static DoubleSupplierBuilder fromRightX(XboxController controller) {
		
		return new DoubleSupplierBuilder(controller::getRightX);
		
	}
	
	public static DoubleSupplierBuilder fromRightX(
		CommandXboxController controller
	) {
		
		return new DoubleSupplierBuilder(controller::getRightX);
		
	}
	
	public static DoubleSupplierBuilder fromRightY(XboxController controller) {
		
		return new DoubleSupplierBuilder(controller::getRightY);
		
	}
	
	public static DoubleSupplierBuilder fromRightY(
		CommandXboxController controller
	) {
		
		return new DoubleSupplierBuilder(controller::getRightY);
		
	}
	
	public DoubleSupplierBuilder withDeadband(double deadband) {
		
		return new DoubleSupplierBuilder(() ->
			DoubleUtilities.applyDeadband(
				this.supplier.getAsDouble(),
				deadband
			)
		);
		
	}
	
	public DoubleSupplierBuilder withScaledDeadband(double deadband) {
		
		return new DoubleSupplierBuilder(() ->
			DoubleUtilities.applyScaledDeadband(
				this.supplier.getAsDouble(),
				deadband
			)
		);
		
	}
	
	public DoubleSupplierBuilder withExponentialCurve(double power) {
		
		return new DoubleSupplierBuilder(() ->
			DoubleUtilities.applyExponentialCurve(
				this.supplier.getAsDouble(),
				power
			)
		);
		
	}
	
	public DoubleSupplierBuilder withClamp(double minimum, double maximum) {
		
		return new DoubleSupplierBuilder(() ->
			DoubleUtilities.applyClamp(
				this.supplier.getAsDouble(),
				minimum,
				maximum
			)
		);
		
	}
	
	public DoubleSupplierBuilder withScaling(double scaling) {
		
		return new DoubleSupplierBuilder(() ->
			this.supplier.getAsDouble() * scaling
		);
		
	}
	
	public DoubleSupplierBuilder withMaximumSlewRate(double limit) {
		
		return new DoubleSupplierBuilder(new DoubleSupplier() {
			
			SlewRateLimiter limiter = new SlewRateLimiter(limit);
			
			@Override
			public double getAsDouble() {
				
				return this.limiter.calculate(
					DoubleSupplierBuilder.this.getAsDouble()
				);
				
			}
			
		});
		
	}
	
	@Override
	public double getAsDouble() {
		
		return this.supplier.getAsDouble();
		
	}
	
}
