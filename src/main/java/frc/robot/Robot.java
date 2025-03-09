// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.configuration.DoublePreference;
import frc.robot.configuration.ReefScoringMode;
import frc.robot.controlschemes.TestingTeleoperativeControlsScheme;
import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
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
    
    CameraServer.startAutomaticCapture();
    
  }

  @Override
  public void robotInit() {
    
    Auton.initializeShuffleboardSelector();
    ReefScoringMode.setMode(ReefScoringMode.DEFAULT);
    
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
