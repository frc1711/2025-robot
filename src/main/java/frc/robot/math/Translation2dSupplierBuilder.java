package frc.robot.util;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.configuration.DoublePreference;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class Translation2dSupplierBuilder implements Supplier<Translation2d> {
	
	protected final Supplier<Translation2d> supplier;
	
	public Translation2dSupplierBuilder(Supplier<Translation2d> initial) {
		
		this.supplier = initial;
		
	}
	
	public Translation2dSupplierBuilder(
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
	public static Translation2dSupplierBuilder fromLeftJoystick(
		XboxController controller
	) {
		
		return new Translation2dSupplierBuilder(
			() -> new Translation2d(controller.getLeftX(), controller.getLeftY())
		);
		
	}
	
	public static Translation2dSupplierBuilder fromLeftJoystick(
		CommandXboxController controller
	) {
		
		return new Translation2dSupplierBuilder(
			() -> new Translation2d(controller.getLeftX(), controller.getLeftY())
		);
		
	}

	/**
	 * Returns a Translation2dSupplierBuilder for controlling the translation of
	 * the drivetrain.
	 *
	 * @param controller The XboxController from which to read the left joystick
	 * input (to use as the translation input).
	 * @return A Translation2dSupplierBuilder for controlling the translation of
	 * the drivetrain.
	 */
	public static Supplier<Translation2d> getTranslationPointSupplier(CommandXboxController controller) {

		return Translation2dSupplierBuilder.fromLeftJoystick(controller)
				.normalizeXboxJoystickToNWU()
				.withClamp(-1, 1)
				.withScaledDeadband(DoublePreference.JOYSTICK_DEADBAND.get())
				.withExponentialCurve(DoublePreference.LINEAR_INPUT_POWER_SMOOTHING.get());

	}
	
	public Translation2dSupplierBuilder swapXY() {
		
		return new Translation2dSupplierBuilder(() -> {
			
			Translation2d original = this.supplier.get();
			
			return new Translation2d(original.getY(), original.getX());
			
		});
		
	}
	
	public Translation2dSupplierBuilder invertX() {
		
		return new Translation2dSupplierBuilder(() -> {
			
			Translation2d original = this.supplier.get();
			
			return new Translation2d(-original.getX(), original.getY());
			
		});
		
	}
	
	public Translation2dSupplierBuilder invertY() {
		
		return new Translation2dSupplierBuilder(() -> {
			
			Translation2d original = this.supplier.get();
			
			return new Translation2d(original.getX(), -original.getY());
			
		});
		
	}
	
	public Translation2dSupplierBuilder normalizeXboxJoystickToNWU() {
		
		return this
			.swapXY()
			.invertX()
			.invertY();
		
	}
	
	public Translation2dSupplierBuilder withDeadband(double deadband) {
		
		return new Translation2dSupplierBuilder(() -> {
			
			Translation2d original = this.supplier.get();
			
			if (original.getNorm() <= 0) return Translation2d.kZero;
			else return new Translation2d(
				DoubleUtilities.applyDeadband(original.getNorm(), deadband),
				original.getAngle()
			);
			
		});
		
	}
	
	public Translation2dSupplierBuilder withScaledDeadband(double deadband) {
		
		return new Translation2dSupplierBuilder(() -> {
			
			Translation2d original = this.supplier.get();
			
			if (original.getNorm() <= 0) return Translation2d.kZero;
			else return new Translation2d(
				DoubleUtilities.applyScaledDeadband(original.getNorm(), deadband),
				original.getAngle()
			);
			
		});
		
	}
	
	public Translation2dSupplierBuilder withExponentialCurve(double exponent) {
		
		return new Translation2dSupplierBuilder(() -> {
			
			Translation2d original = this.supplier.get();
			
			if (original.getNorm() <= 0) return Translation2d.kZero;
			else return new Translation2d(
				DoubleUtilities.applyExponentialCurve(original.getNorm(), exponent),
				original.getAngle()
			);
			
		});
		
	}
	
	public Translation2dSupplierBuilder withClamp(double minimum, double maximum) {
		
		return new Translation2dSupplierBuilder(() -> {
			
			Translation2d original = this.supplier.get();
			
			if (original.getNorm() <= 0) return Translation2d.kZero;
			else return new Translation2d(
				DoubleUtilities.applyClamp(original.getNorm(), minimum, maximum),
				original.getAngle()
			);
			
		});
		
	}
	
	public Translation2dSupplierBuilder withScaling(double scaling) {
		
		return new Translation2dSupplierBuilder(() -> {
			
			Translation2d original = this.supplier.get();
			
			if (original.getNorm() <= 0) return Translation2d.kZero;
			else return new Translation2d(
				original.getNorm() * scaling,
				original.getAngle()
			);
			
		});
		
	}
	
	public Translation2dSupplierBuilder withMaximumSlewRate(double limit) {

		return new Translation2dSupplierBuilder(new Supplier<>() {
			SlewRateLimiter xLimiter = new SlewRateLimiter(limit);
			SlewRateLimiter yLimiter = new SlewRateLimiter(limit);
			@Override
			public Translation2d get() {
				Translation2d original = Translation2dSupplierBuilder.this.get();
				return new Translation2d(
					Inches.of(this.xLimiter.calculate(original.getMeasureX().in(Inches))),
					Inches.of(this.yLimiter.calculate(original.getMeasureY().in(Inches)))
				);
			}
		});
		
	}
	
	@Override
	public Translation2d get() {
		
		return this.supplier.get();
		
	}
	
}
