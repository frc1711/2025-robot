package frc.robot.util;

/**
 * A collection of reusable utility methods for common controls-related
 * operations.
 */
public class ControlsUtilities {
    
    // Prevent this class from being instantiated.
	private ControlsUtilities() {}
	
	public static double normalizeToRange(double input, double minimum, double maximum) {
		
		double delta = maximum - minimum;
		
		return minimum + ((input - minimum + delta) % delta);
		
	}
	
	/**
	 * Applies an exponential curve to the input value, raising it to the
	 * specified exponent and preserving its sign.
	 *
	 * @param input The input value.
	 * @param exponent The exponent to which to raise the input value.
	 * @return The input value raised to the specified exponent, preserving its
	 * sign.
	 */
	public static double applyExponentialCurve(double input, double exponent) {
		
		return Math.copySign(Math.pow(input, exponent), input);
		
	}
	
	/**
	 * Returns the input value so long as its absolute value is greater than the
	 * specified deadband. If the absolute value of the input value is less than
	 * the specified deadband, 0 will be returned.
	 * 
	 * @param input The input value.
	 * @param deadband The deadband.
	 * @return The input value, or 0 if the absolute value of the input value is
	 * less than the specified deadband.
	 */
	public static double applyDeadband(double input, double deadband) {

		return Math.abs(input) < deadband ? 0 : input;

	}
	
	public static Point applyDeadband(Point point, double deadband) {
		
		double hypotenuse = Math.sqrt(Math.pow(point.x, 2) + Math.pow(point.y, 2));
		
		if (Math.abs(hypotenuse) < deadband) return new Point(0, 0);
		else return point;
		
	}
	
	public static double applyScaledDeadband(double input, double deadband) {
		
		return Math.abs(input) < deadband ? 0 : Math.copySign(
			(Math.abs(input) - deadband) / (1 - deadband),
			input
		);
		
	}
	
	public static double applyClamp(
		double input,
		double minimum,
		double maximum
	) {
		
		if (input < minimum) return minimum;
		else if (input > maximum) return maximum;
		else return input;
		
	}
    
	/**
	 * Returns the new value so long as its delta from the old value does not
     * exceed the specified maximum permissible amount. If this delta is greater
	 * than what is permissible, the returned value will instead be the old
	 * value plus the maximum permissible delta in the direction of the new
	 * value.
	 * 
	 * @param oldValue The old value of the variable.
	 * @param newValue The new value of the variable.
	 * @param maxDelta The maximum allowable change.
	 * @return The new value, or the old value plus the maximum permissible
     * delta in the direction of the new value, if the delta between the old and
     * new values exceeds what is permissible.
	 */
	public static double enforceMaximumDelta(
		double oldValue,
		double newValue,
		double maxDelta
	) {
        
		double change = newValue - oldValue;
        
		if (Math.abs(change) <= maxDelta) return newValue;
		else return oldValue + Math.copySign(maxDelta, change);
        
	}
    
    /**
     * Returns the new value so long as it is either moving towards zero, or its
	 * delta from the old value does not exceed the specified maximum
	 * permissible amount. If this delta is greater than what is permissible,
	 * the returned value will instead be the old value plus the maximum
	 * permissible delta in the direction of the new value.
     * 
     * @param oldValue The old value of the variable.
     * @param newValue The new value of the variable.
     * @param maxIncrease The maximum allowable increase.
     * @return The new value, or the old value plus the maximum permissible
	 * delta in the direction of the new value, if the delta between the old and
	 * new values exceeds what is permissible and the new value is not moving
	 * towards zero.
     */
	public static double enforceMaximumPositiveDelta(
		double oldValue,
		double newValue,
		double maxIncrease
	) {
        
		boolean isValueIncreasing = newValue > oldValue;
		boolean isMovementTowardsZero =
			(isValueIncreasing && oldValue < 0) ||
			(!isValueIncreasing && oldValue > 0);
        
		if (isMovementTowardsZero) return newValue;
		
		return isValueIncreasing ?
			Math.min(newValue, oldValue + maxIncrease) :
			Math.max(newValue, oldValue - maxIncrease);
        
	}
    
}
