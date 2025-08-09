package frc.robot.math;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.configuration.DoublePreference;

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

	/**
	 * Returns a DoubleSupplier for controlling the rotation of the drivetrain.
	 *
	 * @param controller The XboxController from which to read the right
	 * joystick input (to use as the rotation input).
	 * @return A DoubleSupplier for controlling the rotation of the drivetrain.
	 */
	public static DoubleSupplierBuilder getRotationDoubleSupplier(CommandXboxController controller) {

		return DoubleSupplierBuilder.fromRightX(controller)
			.withScaling(-1)
			.withClamp(-1, 1)
			.withScaledDeadband(DoublePreference.JOYSTICK_DEADBAND.get())
			.withExponentialCurve(DoublePreference.LINEAR_INPUT_POWER_SMOOTHING.get());

	}

	public DoubleSupplierBuilder withDeadband(double deadband) {

		return new DoubleSupplierBuilder(() -> DoubleUtilities.applyDeadband(
				this.supplier.getAsDouble(),
				deadband
		));

	}

	public DoubleSupplierBuilder withDeadband(DoubleSupplier deadband) {

		return new DoubleSupplierBuilder(() -> DoubleUtilities.applyDeadband(
			this.supplier.getAsDouble(),
			deadband.getAsDouble()
		));

	}
	
	public DoubleSupplierBuilder withScaledDeadband(double deadband) {
		
		return new DoubleSupplierBuilder(() ->
			DoubleUtilities.applyScaledDeadband(
				this.supplier.getAsDouble(),
				deadband
			)
		);
		
	}

	public DoubleSupplierBuilder withScaledDeadband(DoubleSupplier deadband) {

		return new DoubleSupplierBuilder(() ->
			DoubleUtilities.applyScaledDeadband(
				this.supplier.getAsDouble(),
				deadband.getAsDouble()
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

	public DoubleSupplierBuilder withExponentialCurve(DoubleSupplier power) {

		return new DoubleSupplierBuilder(() ->
			DoubleUtilities.applyExponentialCurve(
				this.supplier.getAsDouble(),
				power.getAsDouble()
			)
		);

	}

	public DoubleSupplierBuilder withClamp(double minimum, double maximum) {

		return new DoubleSupplierBuilder(() -> DoubleUtilities.applyClamp(
				this.supplier.getAsDouble(),
				minimum,
				maximum
		));

	}

	public DoubleSupplierBuilder withClamp(
		DoubleSupplier minimum,
		DoubleSupplier maximum
	) {

		return new DoubleSupplierBuilder(() -> DoubleUtilities.applyClamp(
			this.supplier.getAsDouble(),
			minimum.getAsDouble(),
			maximum.getAsDouble()
		));

	}
	
	public DoubleSupplierBuilder withScaling(double scaling) {
		
		return new DoubleSupplierBuilder(() ->
			this.supplier.getAsDouble() * scaling
		);
		
	}

	public DoubleSupplierBuilder withScaling(DoubleSupplier scaling) {

		return new DoubleSupplierBuilder(() ->
			this.supplier.getAsDouble() * scaling.getAsDouble()
		);

	}
	
	public DoubleSupplierBuilder withMaximumSlewRate(double limit) {

		SlewRateLimiter limiter = new SlewRateLimiter(limit);
		
		return new DoubleSupplierBuilder(
			() -> limiter.calculate(DoubleSupplierBuilder.this.getAsDouble())
		);
		
	}
	
	@Override
	public double getAsDouble() {
		
		return this.supplier.getAsDouble();
		
	}
	
}
