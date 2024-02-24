// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SwerveUtil;


public class TeleopSwerve extends Command {

  SwerveSubsystem m_swerveSubsystem;

  DoubleSupplier m_driveX;
  DoubleSupplier m_driveY;
  DoubleSupplier m_rotationX;

  Boolean press;

  
  public TeleopSwerve(SwerveSubsystem swerveSubsystem, DoubleSupplier driveX, DoubleSupplier driveY, DoubleSupplier rotationX, Boolean press) {
    m_swerveSubsystem = swerveSubsystem;
    addRequirements(m_swerveSubsystem);
    m_driveX = driveX;
    m_driveY = driveY;
    m_rotationX = rotationX;
    this.press = press;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.m_swerveSubsystem.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("robot angle", this.m_swerveSubsystem.getHeading());
    SmartDashboard.putNumber("Front Right Turn Velocity", m_swerveSubsystem.m_frontRightModule.getTurnVelocity());
    SmartDashboard.putNumber("Front Right Drive Velocity", m_swerveSubsystem.m_frontRightModule.getDriveVelocity());
    SmartDashboard.putNumber("Front Left Turn Velocity", m_swerveSubsystem.m_frontLeftModule.getTurnVelocity());
    SmartDashboard.putNumber("Front Left Drive Velocity", m_swerveSubsystem.m_frontLeftModule.getDriveVelocity());
    SmartDashboard.putNumber("Back Right Turn Velocity", m_swerveSubsystem.m_backRightModule.getTurnVelocity());
    SmartDashboard.putNumber("Back Right Drive Velocity", m_swerveSubsystem.m_backRightModule.getDriveVelocity());
    SmartDashboard.putNumber("Back Left Turn Velocity", m_swerveSubsystem.m_backLeftModule.getTurnVelocity());
    SmartDashboard.putNumber("Back Left Drive Velocity", m_swerveSubsystem.m_backLeftModule.getDriveVelocity());
    
    double kDriveX = 0;
    double kDriveY = 0;

    double kRotation = 0;

    if (SwerveUtil.handleDrive(m_driveX.getAsDouble(), this.m_driveY.getAsDouble()))
      kDriveX = m_driveX.getAsDouble();
      kDriveY = m_driveY.getAsDouble();
    
    if (SwerveUtil.handleTurn(m_rotationX.getAsDouble())) {
      kRotation = m_rotationX.getAsDouble();
    }

    this.m_swerveSubsystem.setSwerveModuleStates(kDriveX, kDriveY, kRotation * 2);
    if (Math.abs(kDriveX) > 0.1 || Math.abs(kDriveY) > 0.1 || Math.abs(kRotation) > 0.1) {
      moveSwerveModule(this.m_swerveSubsystem.m_backLeftModuleState, this.m_swerveSubsystem.m_backLeftModule);
      moveSwerveModule(this.m_swerveSubsystem.m_backRightModuleState, this.m_swerveSubsystem.m_backRightModule);
      moveSwerveModule(this.m_swerveSubsystem.m_frontLeftModuleState, this.m_swerveSubsystem.m_frontLeftModule);
      moveSwerveModule(this.m_swerveSubsystem.m_frontRightModuleState, this.m_swerveSubsystem.m_frontRightModule);
    }
    else {
      this.m_swerveSubsystem.m_backLeftModule.move(0, 0);
      this.m_swerveSubsystem.m_backRightModule.move(0, 0);
      this.m_swerveSubsystem.m_frontLeftModule.move(0, 0);
      this.m_swerveSubsystem.m_frontRightModule.move(0, 0);
    }
  }

  public void moveSwerveModule(SwerveModuleState state, SwerveModule mod) {
    SmartDashboard.putNumber(mod.m_name + " state speed: ", state.speedMetersPerSecond);
    double driveSpeed = mod.getDriveMotorSpeed(state.speedMetersPerSecond * DriveConstants.driveVelConvFactor * 3.18 * 0.5);
    double goalAngle = state.angle.getDegrees() + 180;
    double ogD = goalAngle - mod.getTurnMotorPosition();
    double d = SwerveUtil.getDisplacement(goalAngle, mod.getTurnMotorPosition());
    if (!(Math.abs(goalAngle - mod.getTurnMotorPosition()) < 2.5)) {
      double turnVoltage = mod.getTurnMotorSpeed(d);
      double newDriveSpeed = Math.abs(ogD) > 90 && Math.abs(ogD) < 270 ? -driveSpeed : driveSpeed;
      mod.move(newDriveSpeed, turnVoltage);
    } else {
      double turnSpeed = 0;
      mod.move(driveSpeed, turnSpeed);
    }
  }
}
