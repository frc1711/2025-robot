package frc.robot.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class PointSupplierBuilder implements Supplier<Point> {
	
	protected final Supplier<Point> supplier;
	
	public PointSupplierBuilder(Supplier<Point> initial) {
		
		this.supplier = initial;
		
	}
	
	public PointSupplierBuilder(
		DoubleSupplier xSupplier,
		DoubleSupplier ySupplier
	) {
		
		this.supplier = () -> new Point(
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
			() -> new Point(controller.getLeftX(), controller.getLeftY())
		);
		
	}
	
	public static PointSupplierBuilder fromLeftJoystick(
		CommandXboxController controller
	) {
		
		return new PointSupplierBuilder(
			() -> new Point(controller.getLeftX(), controller.getLeftY())
		);
		
	}
	
	public PointSupplierBuilder swapXY() {
		
		return new PointSupplierBuilder(() -> {
			
			Point original = this.supplier.get();
			
			return new Point(original.y, original.x);
			
		});
		
	}
	
	public PointSupplierBuilder invertX() {
		
		return new PointSupplierBuilder(() -> {
			
			Point original = this.supplier.get();
			
			return new Point(-original.x, original.y);
			
		});
		
	}
	
	public PointSupplierBuilder invertY() {
		
		return new PointSupplierBuilder(() -> {
			
			Point original = this.supplier.get();
			
			return new Point(original.x, -original.y);
			
		});
		
	}
	
	public PointSupplierBuilder normalizeXboxJoystickToNWU() {
		
		return this
			.swapXY()
			.invertX()
			.invertY();
		
	}
	
	public PointSupplierBuilder withDeadband(double deadband) {
		
		return new PointSupplierBuilder(
			() -> ControlsUtilities.applyDeadband(
				this.supplier.get(),
				deadband
			)
		);
		
	}
	
	public PointSupplierBuilder withScaledDeadband(double deadband) {
		
		return new PointSupplierBuilder(() -> {
			
			Point original = this.supplier.get();
			
			return new Point(
				ControlsUtilities.applyScaledDeadband(original.x, deadband),
				ControlsUtilities.applyScaledDeadband(original.y, deadband)
			);
			
		});
		
	}
	
	public PointSupplierBuilder withExponentialCurve(double exponent) {
		
		return new PointSupplierBuilder(() -> {
			
			Point original = this.supplier.get();
			
			return new Point(
				ControlsUtilities.applyExponentialCurve(original.x, exponent),
				ControlsUtilities.applyExponentialCurve(original.y, exponent)
			);
			
		});
		
	}
	
	public PointSupplierBuilder withClamp(double minimum, double maximum) {
		
		return new PointSupplierBuilder(() -> {
			
			Point original = this.supplier.get();
			
			return new Point(
				ControlsUtilities.applyClamp(original.x, minimum, maximum),
				ControlsUtilities.applyClamp(original.y, minimum, maximum)
			);
			
		});
		
	}
	
	public PointSupplierBuilder withScaling(double scaling) {
		
		return new PointSupplierBuilder(() -> {
			
			Point original = this.supplier.get();
			
			return new Point(
				original.x * scaling,
				original.y * scaling
			);
			
		});
		
	}
	
	@Override
	public Point get() {
		
		return this.supplier.get();
		
	}
	
}
