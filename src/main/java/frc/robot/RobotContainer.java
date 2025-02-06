// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.LedSubsystem;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final LedSubsystem m_exampleSubsystem = new LedSubsystem();
  
  private final Spark spark = new Spark(2);

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }
  
  private void configureBindings() {
    m_driverController.b().whileTrue(m_exampleSubsystem.changeLeds(spark, 0.01));
    m_driverController.a().whileTrue(m_exampleSubsystem.changeLeds(spark, 0.61));
  }
}
