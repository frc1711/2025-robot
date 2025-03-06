// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.configuration.SwerveModuleConfiguration;
import frc.robot.devicewrappers.RaptorsNavX;
import frc.robot.util.Point;
import frc.robot.util.SwerveModule;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Stream;

import static edu.wpi.first.units.Units.*;

public class Swerve extends SubsystemBase {
	
	protected static final double HEADING_PID_kP = 0.01;
	
	protected static final Angle HEADING_TOLERANCE = Degrees.of(2);
	
	public final SwerveModule[] modules;
	
	protected final PIDController headingPIDController;
	
	protected final RaptorsNavX gyro;
	
	protected final SwerveDriveKinematics kinematics;
	
	protected boolean isHeadingLockEnabled;
	
	protected ChassisSpeeds chassisSpeeds;
	
	public final Swerve.Commands commands;
	
	public Swerve() {
		
		this.modules = SwerveModuleConfiguration.getModuleConfigurations()
			.map(SwerveModule::new)
			.toArray(SwerveModule[]::new);
		
		this.headingPIDController = new PIDController(0, 0, 0);
		this.gyro = new RaptorsNavX();
		this.kinematics = new SwerveDriveKinematics(
			SwerveModuleConfiguration.getModuleConfigurations()
				.map(config -> config.positionInRobot)
				.toArray(Translation2d[]::new)
		);
		this.isHeadingLockEnabled = false;
		this.commands = new Swerve.Commands();
		this.chassisSpeeds = new ChassisSpeeds(0, 0, 0);
		
		this.headingPIDController.enableContinuousInput(0, 360);
		
		this.calibrateFieldRelativeHeading();
		
		ShuffleboardTab shuffleboardCalibrationTab =
			Shuffleboard.getTab("Calibration");
		
		shuffleboardCalibrationTab.add(
			this.commands.calibrateModuleSteeringHeadings()
		);
		
		shuffleboardCalibrationTab.add(
			this.commands.calibrateFieldRelativeHeading()
		);
		
		SmartDashboard.putData("Swerve Drive", this.getSwerveStateSendable());
		
	}
	
	protected Stream<SwerveModule> getModuleStream() {
		
		return Stream.of(this.modules);
		
	}
	
	protected void applyModuleStates(SwerveModuleState[] moduleStates) {
		
		this.getModuleStream().forEach(module -> {
			module.updateModuleState(moduleStates[module.getID()]);
		});
		
	}
	
	public void stop() {
		
		this.applyChassisSpeeds(new ChassisSpeeds(0, 0, 0), false);
		
	}
	
	public void calibrateFieldRelativeHeading() {
		
		this.calibrateFieldRelativeHeading(Degrees.of(0));
		
	}
	
	public void calibrateFieldRelativeHeading(Angle currentHeading) {
		
		this.gyro.calibrate(currentHeading);
		this.setFieldRelativeHeadingSetpoint(currentHeading.times(-1));
		
	}
	
	public Angle getFieldRelativeHeading() {
		
		return this.gyro.getRotation();
		
	}
	
	public void applyChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean fieldRelative) {
		
		if (fieldRelative) {
			
			chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
				chassisSpeeds,
				Rotation2d.fromDegrees(this.gyro.getRotation().in(Degrees))
			);
			
		}

//		// Poll the current state of the heading lock.
//		boolean wasHeadingLockEnabled = this.isHeadingLockEnabled;
//
//		// Enable the heading lock if we are not receiving any rotational input,
//		// otherwise, disable it (if we *are* receiving rotational input).
//		this.isHeadingLockEnabled = Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= 0;
//
//		// Check for a rising edge of the heading lock state.
//		boolean didHeadingLockBecomeEnabled = (
//			!wasHeadingLockEnabled &&
//			this.isHeadingLockEnabled
//		);
//
//		// If the heading lock *became* active...
//		if (didHeadingLockBecomeEnabled) {
//
//			// Update the heading setpoint to the heading we've rotated to while
//			// the heading lock was disabled.
//			this.headingPIDController.setSetpoint(
//				this.getFieldRelativeHeading().in(Degrees)
//			);
//
//		}

//		chassisSpeeds.omegaRadiansPerSecond *= 1.5;
		
		// Update the chassis speeds.
		this.chassisSpeeds = chassisSpeeds;
		
	}
	
	public void setFieldRelativeHeadingSetpoint(Angle heading) {
		
		this.headingPIDController.setSetpoint(heading.in(Degrees));
		
	}
	
	@Override
	public void periodic() {
		
		double headingPIDOutput = this.headingPIDController.calculate(
			this.getFieldRelativeHeading().in(Degrees)
		);

		this.isHeadingLockEnabled = false;

		ChassisSpeeds newChassisSpeeds = new ChassisSpeeds(
			this.chassisSpeeds.vxMetersPerSecond,
			this.chassisSpeeds.vyMetersPerSecond,
			this.isHeadingLockEnabled ? headingPIDOutput : this.chassisSpeeds.omegaRadiansPerSecond
		);

		this.applyModuleStates(
			this.kinematics.toSwerveModuleStates(newChassisSpeeds)
		);
		
	}
	
	public SysIdRoutine getDriveMotorsSysIdRoutine() {
		
		return new SysIdRoutine(
			new SysIdRoutine.Config(),
			new SysIdRoutine.Mechanism(
				(voltage) -> this.getModuleStream().forEach(module -> {
					module.driveMotorController.setVoltage(voltage);
				}),
				null,
				this
			)
		);
		
	}
	
	@Override
	public void initSendable(SendableBuilder builder) {
		
		builder.addDoubleProperty(
			"Heading",
			() -> this.getFieldRelativeHeading().in(Degrees),
			(double headingDegrees) -> this.setFieldRelativeHeadingSetpoint(Degrees.of(headingDegrees))
		);
		
		builder.addDoubleProperty(
			"Heading Setpoint",
			() -> this.headingPIDController.getSetpoint(),
			(double headingDegrees) -> this.setFieldRelativeHeadingSetpoint(Degrees.of(headingDegrees))
		);
		
	}
	
	public Sendable getSwerveStateSendable() {
		
		return builder -> {
			
			AngleUnit angleUnit = Degrees;
			LinearVelocityUnit velocityUnit = MetersPerSecond;
			
			builder.setSmartDashboardType("SwerveDrive");
			
			SwerveModule frontLeft =
				this.modules[SwerveModuleConfiguration.FRONT_LEFT.moduleID];
			SwerveModule frontRight =
				this.modules[SwerveModuleConfiguration.FRONT_RIGHT.moduleID];
			SwerveModule backLeft =
				this.modules[SwerveModuleConfiguration.REAR_LEFT.moduleID];
			SwerveModule backRight =
				this.modules[SwerveModuleConfiguration.REAR_RIGHT.moduleID];
			
			builder.addDoubleProperty(
				"Front Left Angle",
				() -> frontLeft.getSteeringHeading().in(angleUnit),
				null
			);
			
			builder.addDoubleProperty(
				"Front Left Velocity",
				() -> frontLeft.getVelocity().in(velocityUnit),
				null
			);
			
			builder.addDoubleProperty(
				"Front Right Angle",
				() -> frontRight.getSteeringHeading().in(angleUnit),
				null
			);
			
			builder.addDoubleProperty(
				"Front Right Velocity",
				() -> frontRight.getVelocity().in(velocityUnit),
				null
			);
			
			builder.addDoubleProperty(
				"Back Left Angle",
				() -> backLeft.getSteeringHeading().in(angleUnit),
				null
			);
			
			builder.addDoubleProperty(
				"Back Left Velocity",
				() -> backLeft.getVelocity().in(velocityUnit),
				null
			);
			
			builder.addDoubleProperty(
				"Back Right Angle",
				() -> backRight.getSteeringHeading().in(angleUnit),
				null
			);
			
			builder.addDoubleProperty(
				"Back Right Velocity",
				() -> backRight.getVelocity().in(velocityUnit),
				null
			);
			
			builder.addDoubleProperty(
				"Robot Angle",
				() -> this.getFieldRelativeHeading().in(angleUnit),
				null
			);
			
		};
		
	}
	
	public class Commands {
		
		public Command calibrateModuleSteeringHeadings() {

			return Swerve.this
				.runOnce(() -> Swerve.this.getModuleStream().forEach(SwerveModule::calibrateSteeringHeading))
				.withName("Calibrate Swerve Module Steering Headings")
				.ignoringDisable(true);

		}
		
		public Command calibrateFieldRelativeHeading() {
			
			return this.calibrateFieldRelativeHeading(Degrees.of(0));
			
		}
		
		public Command calibrateFieldRelativeHeading(Angle currentHeading) {
			
			return Swerve.this
				.runOnce(() -> Swerve.this.calibrateFieldRelativeHeading(currentHeading))
				.withName("Calibrate Swerve Field-relative Heading")
				.ignoringDisable(true);
			
		}
		
		public Command setFieldRelativeHeading(Angle heading) {
			
			return Swerve.this.runOnce(
				() -> Swerve.this.setFieldRelativeHeadingSetpoint(heading)
			);
			
		}
		
		public Command driveFieldRelative(Supplier<Point> xy, DoubleSupplier rotation) {
			
			return Swerve.this.run(() -> {
				
				Point xyPoint = xy.get();
				
				Swerve.this.applyChassisSpeeds(
					new ChassisSpeeds(
						xyPoint.x,
						xyPoint.y,
						rotation.getAsDouble()
					),
//					true
					false
				);
				
			});
			
		}
		
		public Command driveRobotRelative(Supplier<Point> xy, DoubleSupplier rotation) {
			
			return Swerve.this.run(() -> {
				
				Point xyPoint = xy.get();
				
				Swerve.this.applyChassisSpeeds(
					new ChassisSpeeds(
						xyPoint.x,
						xyPoint.y,
						rotation.getAsDouble()
					),
					false
				);
				
			});
			
		}
		
		public Command driveForTime(
			Angle translationAngle,
			double translationSpeed,
			Angle heading,
			Time duration
		) {
			
			return new FunctionalCommand(
				() -> Swerve.this.setFieldRelativeHeadingSetpoint(heading),
				() -> Swerve.this.applyChassisSpeeds(new ChassisSpeeds(
					Math.cos(translationAngle.in(Radians)) * translationSpeed,
					Math.sin(translationAngle.in(Radians)) * translationSpeed,
					0
				), true),
				(wasInterrupted) -> Swerve.this.stop(),
				() -> false,
				Swerve.this
			).withTimeout(duration.in(Seconds));
			
		}
		
		public Command driveForTime2(
			Angle translationAngle,
			double translationSpeed,
			Angle heading,
			Time duration
		) {
			
			double rampTimeSeconds = 0.5;
			Timer timer = new Timer();
			
			return new FunctionalCommand(
				() -> {
					timer.start();
					Swerve.this.setFieldRelativeHeadingSetpoint(heading);
				},
				() -> {
					
					double timeSinceStart = timer.get();
					double timeUntilEnd = duration.in(Seconds) - timeSinceStart;
					double activeSpeed = Math.min(
						(timeSinceStart/rampTimeSeconds) * translationSpeed,
						(timeUntilEnd/rampTimeSeconds) * translationSpeed
					);
					
					activeSpeed = Math.min(activeSpeed, translationSpeed);
					
					Swerve.this.applyChassisSpeeds(new ChassisSpeeds(
						Math.cos(translationAngle.in(Radians)) * activeSpeed,
						Math.sin(translationAngle.in(Radians)) * activeSpeed,
						0
					), true);
					
				},
				(wasInterrupted) -> Swerve.this.stop(),
				() -> false,
				Swerve.this
			).withTimeout(duration.in(Seconds));
			
		}
		
		public Command driveTestCommand(LinearVelocity speed) {
			
			return new FunctionalCommand(
				() -> Swerve.this.applyChassisSpeeds(new ChassisSpeeds(
					speed,
					FeetPerSecond.of(0),
					DegreesPerSecond.of(0)
				), false),
				() -> {},
				(wasInterrupted) -> Swerve.this.stop(),
				() -> false,
				Swerve.this
			);
			
		}

//		public SwerveControllerCommand drive(Trajectory trajectory, Rotation2d rotation) {
//
//			return new SwerveControllerCommand(
//				trajectory,
//				Swerve.this::getRobotPose,
//				Swerve.this.kinematics,
//				Swerve.this.controller,
//				() -> rotation,
//				(outputModuleStates) -> {},
//				Swerve.this
//			);
//
//		}
		
		public Command sysIdSteerQuasistatic(SysIdRoutine.Direction direction) {
			
			return Swerve.this.modules[2]
				.getSteerMotorSysIdRoutine()
				.quasistatic(direction);
			
		}
		
		public Command sysIdSteerDynamic(SysIdRoutine.Direction direction) {
			
			return Swerve.this.modules[2]
				.getSteerMotorSysIdRoutine()
				.dynamic(direction);
			
		}
		
		public Command sysIdDriveQuasistatic(SysIdRoutine.Direction direction) {
			
			return Swerve.this.getDriveMotorsSysIdRoutine()
				.quasistatic(direction);
			
		}
		
		public Command sysIdDriveDynamic(SysIdRoutine.Direction direction) {
			
			return Swerve.this.getDriveMotorsSysIdRoutine()
				.dynamic(direction);
			
		}
		
	}
	
}
