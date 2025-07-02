package frc.robot.util;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.LinearAcceleration;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

public class ChassisSpeedsSlewRateLimiter {

    protected final SlewRateLimiterTranslation2D translationSlewRatLimiter;

    protected final SlewRateLimiter angularSlewRateLimiter;

    public ChassisSpeedsSlewRateLimiter(
        LinearAcceleration maxLinearAcceleration,
        AngularAcceleration maxAngularAcceleration
    ) {

        this.translationSlewRatLimiter =
            new SlewRateLimiterTranslation2D(maxLinearAcceleration.in(MetersPerSecondPerSecond));
        this.angularSlewRateLimiter =
            new SlewRateLimiter(maxAngularAcceleration.in(RadiansPerSecondPerSecond));

    }

    public ChassisSpeeds calculate(ChassisSpeeds value) {

        Translation2d translation = this.translationSlewRatLimiter.calculate(new Translation2d(
            value.vxMetersPerSecond,
            value.vyMetersPerSecond
        ));

        double omegaRadiansPerSecond = this.angularSlewRateLimiter.calculate(value.omegaRadiansPerSecond);

        return new ChassisSpeeds(
            translation.getX(),
            translation.getY(),
            omegaRadiansPerSecond
        );

    }

}
