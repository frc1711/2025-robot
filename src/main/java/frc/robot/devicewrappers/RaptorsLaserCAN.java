package frc.robot.devicewrappers;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.configuration.CANDevice;

import static edu.wpi.first.units.Units.Millimeters;

public class RaptorsLaserCAN {
	
	protected final LaserCan laserCAN;
	
	public RaptorsLaserCAN(CANDevice canDevice) {
		
		this.laserCAN = new LaserCan(canDevice.id);
		
	}
	
	public Distance getDistance() {
		
		LaserCanInterface.Measurement measurement =
			this.laserCAN.getMeasurement();
		
		if (measurement == null) return null;
		else return Millimeters.of(measurement.distance_mm);
	
	}
	
	public Trigger getDistanceTrigger(Distance triggerDistance) {
		
		return new Trigger(() -> {
			
			Distance measuredDistance = this.getDistance();
			
			if (measuredDistance == null) return false;
			else return measuredDistance.lte(triggerDistance);
			
		});
		
	}
	
}
