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

public class ElevatorSubsystem extends SubsystemBase {

  private final Encoder elevatorEncoder;

  /** Creates a new ExampleSubsystem.*/
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
          talon.setControl(new CoastOut()); // Sets Motor to coast
          talon.set(speed); // Sets to desired speed
        },
        () -> {
            talon.stopMotor(); // Stops motor
            talon.setControl(new StaticBrake());
        });
  }
 /**
   * This will make the elevator motor move to a certain height
   * @param talon The motor controller
   * @param height The target height of the elevator (555 = ~1 inch)
   * @param speed The speed to get to the target height
   */
  public Command moveElevatorTo(TalonFX talon, int height, int speed) {
    // 555 is about 1 inch
    return startEnd(() -> {
      if (height > elevatorEncoder.get()) {
        talon.set(-(speed)); // Motor to go up
        while (elevatorEncoder.get() >= height) { // It should stop motor once the elevator is higher than the desired height.
          System.out.println(elevatorEncoder.get()); // It doen't rn :(
        }
      } else {
        talon.set(speed); // Sets motor to go down
        while (elevatorEncoder.get() < height) { // It should stop motor once the elevator is lower than the desired
          System.out.println(elevatorEncoder.get()); // It also doen't rn :(
        }
      }
    }, () -> {
      talon.set(0); // Stops the talon
      talon.setControl(new StaticBrake()); // Sets motor control to static brake
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
