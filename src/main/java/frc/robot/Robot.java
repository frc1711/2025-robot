// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.configuration.DoublePreference;
import frc.robot.configuration.ReefAlignmentMode;
import org.littletonrobotics.urcl.URCL;

import static edu.wpi.first.units.Units.Degrees;

public class Robot extends TimedRobot {
  
  protected RobotContainer robotContainer;
  
  protected Command autonCommand;
  
  public Robot() {
    
    this.robotContainer = new RobotContainer();
    
    // Start the CanBridge server for the LaserCANs.
    CanBridge.runTCP();
    
    // Start the URCL (Unofficial REV-Compatible Logger) server.
    // This is used for logging REV device to the CAN network, which is in turn
    // used for system identification.
    DataLogManager.start();
    URCL.start();
    
    DoublePreference.init();
    
//    CameraServer.startAutomaticCapture();
    
  }

  @Override
  public void robotInit() {
    
    Auton.initializeShuffleboardSelector();
    ReefAlignmentMode.setMode(ReefAlignmentMode.DEFAULT);
    this.robotContainer.swerve.calibrateFieldRelativeHeading(Degrees.of(180));
    
  }

  @Override
  public void robotPeriodic() {
    
    CommandScheduler.getInstance().run();
    
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    
    this.autonCommand = Auton.runSelectedAuton(this.robotContainer);
    
  }
  
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    
    if (this.autonCommand != null) this.autonCommand.cancel();
    
    this.robotContainer.configureTeleoperativeControls();
    
  }
  
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    
    CommandScheduler.getInstance().cancelAll();
    
  }
  
  @Override
  public void testPeriodic() {}
  
}
