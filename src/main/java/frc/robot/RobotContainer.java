// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Orchestra;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.configuration.StatusLightsPattern;
import frc.robot.controlschemes.ControlsScheme;
import frc.robot.controlschemes.TestingTeleoperativeControlsScheme;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  
  protected static final ControlsScheme CONTROLS_SCHEME =
      new TestingTeleoperativeControlsScheme();
  
  public final Climber climber;
  
  public final Elevator elevator;
  
  public final Intake intake;
  
  public final Mailbox mailbox;
  
  public final Swerve swerve;
  
  public final StatusLights lights;
  
  public final Orchestra orchestra;
  
  public final ComplexCommands complexCommands;
  
  public final CommandXboxController controller1;
  
  public final CommandXboxController controller2;

  public RobotContainer() {
    
    this.climber = new Climber();
    this.elevator = new Elevator();
    this.intake = new Intake();
    this.mailbox = new Mailbox();
    this.swerve = new Swerve();
    this.lights = new StatusLights();
    this.orchestra = new Orchestra();
    this.complexCommands = new ComplexCommands(this);
    this.controller1 = new CommandXboxController(0);
    this.controller2 = new CommandXboxController(1);
    
    this.lights.set(StatusLightsPattern.SOLID_COLORS_LAWN_GREEN);
    
    Shuffleboard.getTab("Subsystems").add("Climber", this.climber);
    Shuffleboard.getTab("Subsystems").add("Elevator", this.elevator);
    Shuffleboard.getTab("Subsystems").add("Intake", this.intake);
    Shuffleboard.getTab("Subsystems").add("Swerve", this.swerve);
    
    this.setupOrchestra();
    
  }
  
  protected void setupOrchestra() {
    
    this.climber.addToOrchestra(this.orchestra);
    this.elevator.addToOrchestra(this.orchestra);
    this.intake.addToOrchestra(this.orchestra);
    this.mailbox.addToOrchestra(this.orchestra);
    
    orchestra.loadMusic("paint-it-black-2.chrp");
    
  }
  
  public void configureTeleoperativeControls() {
    
    CONTROLS_SCHEME.configureControls(
        this,
        this.controller1,
        this.controller2
    );
    
//    this.controller1.a().whileTrue(this.swerve.commands.sysIdSteerQuasistatic(SysIdRoutine.Direction.kForward));
//    this.controller1.b().whileTrue(this.swerve.commands.sysIdSteerQuasistatic(SysIdRoutine.Direction.kReverse));
//    this.controller1.x().whileTrue(this.swerve.commands.sysIdSteerDynamic(SysIdRoutine.Direction.kForward));
//    this.controller1.y().whileTrue(this.swerve.commands.sysIdSteerDynamic(SysIdRoutine.Direction.kReverse));
  
  }
  
}
