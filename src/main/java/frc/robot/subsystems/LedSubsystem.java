// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public LedSubsystem() {}
  
  /**
   * This will change the leds associated with said spark to a color based on the speed
   * @param spark The motor controller that is used with PWM for the LEDs
   * @param speed The color or pattern the LEDs will be set to
   */
  public Command changeLeds(Spark spark, double speed) {
    return runOnce(
        () -> {
          spark.set(speed);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
