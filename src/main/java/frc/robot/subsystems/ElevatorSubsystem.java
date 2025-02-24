// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ElevatorSubsystem extends SubsystemBase {

  private final Encoder elevatorEncoder;

  /** Creates a new ExampleSubsystem. */
  public ElevatorSubsystem(Encoder encoder) {
    this.elevatorEncoder = encoder;
  }
  
  /**
   * This will make the elevator motor spin at a certain
   * @param talon The motor controller
   * @param speed The speed of the motor
   */
  public Command moveElevator(TalonFX talon, double speed) {
    return startEnd(
        () -> {
          talon.setControl(new CoastOut());
          talon.set(speed);
        },
        () -> {
            talon.stopMotor();
            talon.setControl(new StaticBrake());
        });
  }

  public Command moveElevatorTo(TalonFX talon, int height) {
    // 555 is about 1 inch
    return startEnd(() -> {
      if (height > elevatorEncoder.get()) {
        talon.set(-0.1);
        while (elevatorEncoder.get() >= height) {
          System.out.println(elevatorEncoder.get());
        }
      } else {
        talon.set(0.1);
        while (elevatorEncoder.get() < height) {
          System.out.println(elevatorEncoder.get());
        }
      }
    }, () -> {
      talon.set(0);
      talon.setControl(new StaticBrake());
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
