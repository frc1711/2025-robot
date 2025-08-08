package frc.robot.math;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;

import static edu.wpi.first.units.Units.*;

public class LinearMotionProfiler {
    
    protected final LinearVelocity maxVelocity;
    
    protected final LinearAcceleration acceleration;
    
    protected final LinearAcceleration deceleration;
    
    protected Time lastTime;
    
    public LinearMotionProfiler(
        LinearVelocity maxVelocity,
        LinearAcceleration acceleration
    ) {
        
        this(maxVelocity, acceleration, acceleration);
        
    }
    
    public LinearMotionProfiler(
        LinearVelocity maxVelocity,
        LinearAcceleration acceleration,
        LinearAcceleration deceleration
    ) {
        
        this.maxVelocity = maxVelocity;
        this.acceleration = acceleration.times(
            acceleration.lt(MetersPerSecondPerSecond.of(0)) ? -1 : 1
        );
        this.deceleration = deceleration.times(
            deceleration.lt(MetersPerSecondPerSecond.of(0)) ? -1 : 1
        );
        this.lastTime = Seconds.of(Timer.getFPGATimestamp());
        
    }
    
    public LinearVelocity calculate(
        Distance remainingDistance,
        LinearVelocity currentVelocity
    ) {

        Time currentTime = Seconds.of(Timer.getFPGATimestamp());
        Time deltaTime = currentTime.minus(this.lastTime);
        this.lastTime = currentTime;
        
        LinearVelocity acceleratingVelocity = MetersPerSecond.of(Math.min(
            currentVelocity.plus(this.acceleration.times(deltaTime)).in(MetersPerSecond),
            this.maxVelocity.in(MetersPerSecond)
        ));

        LinearVelocity deceleratingVelocity = MetersPerSecond.of(Math.min(
            Math.sqrt(2 * this.deceleration.in(MetersPerSecondPerSecond) * remainingDistance.in(Meters)),
            this.maxVelocity.in(MetersPerSecond)
        ));
        
        return MetersPerSecond.of(Math.min(
            deceleratingVelocity.in(MetersPerSecond),
            acceleratingVelocity.in(MetersPerSecond)
        ));
        
    }
    
}
