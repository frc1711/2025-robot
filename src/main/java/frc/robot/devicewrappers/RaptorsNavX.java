package frc.robot.devicewrappers;

import com.studica.frc.AHRS;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

public class RaptorsNavX {
	
	protected final AHRS navX;
	
	protected Angle adjustmentAngle;
	
	public RaptorsNavX() {
		
		this.navX = new AHRS(AHRS.NavXComType.kMXP_SPI);
		this.adjustmentAngle = Degrees.of(0);
		
	}
	
	public Angle getRotation() {
		
		Angle result = Degrees.of(
			this.navX.getRotation2d().getDegrees()
		);
		
		result = result.minus(this.adjustmentAngle);
		
		return result;
		
	}
	
	public AngularVelocity getAngularVelocity() {
		
		return DegreesPerSecond.of(this.navX.getRate());
		
	}
	
	public void calibrate() {
	
		this.calibrate(Degrees.of(0));
	
	}
	
	public void calibrate(Angle currentHeading) {
		
		this.navX.reset();
		this.adjustmentAngle = currentHeading;
		
	}
	
}
