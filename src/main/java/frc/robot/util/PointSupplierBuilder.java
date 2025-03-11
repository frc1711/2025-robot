package frc.robot.util;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class PointSupplierBuilder implements Supplier<Translation2d> {
	
	protected final Supplier<Translation2d> supplier;
	
	public PointSupplierBuilder(Supplier<Translation2d> initial) {
		
		this.supplier = initial;
		
	}
	
	public PointSupplierBuilder(
		DoubleSupplier xSupplier,
		DoubleSupplier ySupplier
	) {
		
		this.supplier = () -> new Translation2d(
			xSupplier.getAsDouble(),
			ySupplier.getAsDouble()
		);
		
	}
	
	/**
	 * Creates a new PointSupplierBuilder from the left joystick of the given
	 * XboxController.
	 *
	 * This method normalizes the joystick input to the standard FRC
	 * 'North West Up' ('NMU') coordinate system.
	 *
	 * @param controller The XboxController from which to read the left joystick
	 * input.
	 */
	public static PointSupplierBuilder fromLeftJoystick(
		XboxController controller
	) {
		
		return new PointSupplierBuilder(
			() -> new Translation2d(controller.getLeftX(), controller.getLeftY())
		);
		
	}
	
	public static PointSupplierBuilder fromLeftJoystick(
		CommandXboxController controller
	) {
		
		return new PointSupplierBuilder(
			() -> new Translation2d(controller.getLeftX(), controller.getLeftY())
		);
		
	}
	
	public PointSupplierBuilder swapXY() {
		
		return new PointSupplierBuilder(() -> {
			
			Translation2d original = this.supplier.get();
			
			return new Translation2d(original.getY(), original.getX());
			
		});
		
	}
	
	public PointSupplierBuilder invertX() {
		
		return new PointSupplierBuilder(() -> {
			
			Translation2d original = this.supplier.get();
			
			return new Translation2d(-original.getX(), original.getY());
			
		});
		
	}
	
	public PointSupplierBuilder invertY() {
		
		return new PointSupplierBuilder(() -> {
			
			Translation2d original = this.supplier.get();
			
			return new Translation2d(original.getX(), -original.getY());
			
		});
		
	}
	
	public PointSupplierBuilder normalizeXboxJoystickToNWU() {
		
		return this
			.swapXY()
			.invertX()
			.invertY();
		
	}
	
	public PointSupplierBuilder withDeadband(double deadband) {
		
		return new PointSupplierBuilder(() -> {
			
			Translation2d original = this.supplier.get();
			
			if (original.getNorm() <= 0) return Translation2d.kZero;
			else return new Translation2d(
				ControlsUtilities.applyDeadband(original.getNorm(), deadband),
				original.getAngle()
			);
			
		});
		
	}
	
	public PointSupplierBuilder withScaledDeadband(double deadband) {
		
		return new PointSupplierBuilder(() -> {
			
			Translation2d original = this.supplier.get();
			
			if (original.getNorm() <= 0) return Translation2d.kZero;
			else return new Translation2d(
				ControlsUtilities.applyScaledDeadband(original.getNorm(), deadband),
				original.getAngle()
			);
			
		});
		
	}
	
	public PointSupplierBuilder withExponentialCurve(double exponent) {
		
		return new PointSupplierBuilder(() -> {
			
			Translation2d original = this.supplier.get();
			
			if (original.getNorm() <= 0) return Translation2d.kZero;
			else return new Translation2d(
				ControlsUtilities.applyExponentialCurve(original.getNorm(), exponent),
				original.getAngle()
			);
			
		});
		
	}
	
	public PointSupplierBuilder withClamp(double minimum, double maximum) {
		
		return new PointSupplierBuilder(() -> {
			
			Translation2d original = this.supplier.get();
			
			if (original.getNorm() <= 0) return Translation2d.kZero;
			else return new Translation2d(
				ControlsUtilities.applyClamp(original.getNorm(), minimum, maximum),
				original.getAngle()
			);
			
		});
		
	}
	
	public PointSupplierBuilder withScaling(double scaling) {
		
		return new PointSupplierBuilder(() -> {
			
			Translation2d original = this.supplier.get();
			
			if (original.getNorm() <= 0) return Translation2d.kZero;
			else return new Translation2d(
				original.getNorm() * scaling,
				original.getAngle()
			);
			
		});
		
	}
	
	public PointSupplierBuilder withMaximumSlewRate(double limit) {
		
		return new PointSupplierBuilder(new Supplier<Translation2d>() {
			SlewRateLimiter xLimiter = new SlewRateLimiter(limit);
			SlewRateLimiter yLimiter = new SlewRateLimiter(limit);
			@Override
			public Translation2d get() {
				
				Translation2d original = PointSupplierBuilder.this.get();
				
				return new Translation2d(
					this.xLimiter.calculate(original.getX()),
					this.yLimiter.calculate(original.getY())
				);
				
			}
		});
		
	}
	
	@Override
	public Translation2d get() {
		
		return this.supplier.get();
		
	}
	
}
