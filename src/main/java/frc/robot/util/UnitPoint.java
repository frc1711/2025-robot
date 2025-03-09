package frc.robot.util;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

/**
 * A class representative of an x, y point in 2D space.
 */
public class UnitPoint<T extends Measure<? extends Unit>> {
    
    /**
     * The X coordinate of this point in 2D space.
     */
    protected T x;
    
    /**
     * The Y coordinate of this point in 2D space.
     */
    protected T y;
    
    /**
     * Initializes a new Point instance at the given X, Y coordinates.
     *
     * @param x The X coordinate of this point in 2D space.
     * @param y The Y coordinate of this point in 2D space.
     */
    public UnitPoint(T x, T y) {

        this.x = x;
        this.y = y;

    }

}
