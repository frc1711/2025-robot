import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.VirtualField;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.*;

public class VirtualFieldTest {
	
	private static Stream<Arguments> getReefCenterPointTestCases() {
		
		return Stream.of(
			Arguments.of(DriverStation.Alliance.Red, new Translation2d(0, 4.0259)),
			Arguments.of(DriverStation.Alliance.Blue, new Translation2d(4.697476, 4.0259))
		);
		
	}
	
	@ParameterizedTest
	@MethodSource("getReefCenterPointTestCases")
	public void doesGetReefCenterPointWork(
		DriverStation.Alliance alliance,
		Translation2d expectedCenterPoint
	) {
		
		Translation2d actualCenterPoint = VirtualField.getReefCenterPoint(alliance);
		
		assertEquals(
			expectedCenterPoint.getX(),
			actualCenterPoint.getX(),
			"Expected center point X value to be " + expectedCenterPoint.getX() + " but got " + actualCenterPoint.getX()
		);
		
		assertEquals(
			expectedCenterPoint.getY(),
			actualCenterPoint.getY(),
			"Expected center point Y value to be " + expectedCenterPoint.getY() + " but got " + actualCenterPoint.getY()
		);
		
	}
	
}
