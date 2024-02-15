// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SwerveUtil;

public class OldTeleopSwerve extends CommandBase {
  public SwerveSubsystem m_swerveSubsystem;

  public DoubleSupplier m_driveX;
  public DoubleSupplier m_driveY;
  public DoubleSupplier m_rotationX;
  public DoubleSupplier m_rotationY;
  
  public double turnSpeed = 0;
  public double swerveModuleAngleDisplacement = 0;

  public OldTeleopSwerve(SwerveSubsystem swerveSubsystem, DoubleSupplier driveX, DoubleSupplier driveY, DoubleSupplier rotationX, DoubleSupplier rotationY) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.m_driveX = driveX;
    this.m_driveY = driveY;
    this.m_rotationX = rotationX;
    this.m_rotationY = rotationY;
    addRequirements(this.m_swerveSubsystem);
  }

  @Override
  public void initialize() {
    this.m_swerveSubsystem.resetGyro();
  }

  @Override
  public void execute() {
    double rotationAngle;
    if (Math.abs(this.m_rotationX.getAsDouble()) >= .1 || Math.abs(this.m_rotationY.getAsDouble()) >= .1) 
      rotationAngle = -SwerveUtil.getAngle(this.m_rotationX.getAsDouble(), this.m_rotationY.getAsDouble(), m_swerveSubsystem.getHeading()) + 180;
    else rotationAngle = 0;

    double rotationSpeed = this.m_rotationX.getAsDouble() * 0.3;
    SmartDashboard.putNumber("robot angle", this.m_swerveSubsystem.getHeading());
    if (Math.abs(this.m_driveX.getAsDouble()) >= 0.1 || Math.abs(this.m_driveY.getAsDouble()) >= 0.1 || Math.abs(this.m_rotationX.getAsDouble()) >= .1 || Math.abs(this.m_rotationY.getAsDouble()) >= .1) {
      this.m_swerveSubsystem.setSwerveModuleStates(this.m_driveX.getAsDouble(), this.m_driveY.getAsDouble(), rotationSpeed);
      moveSwerveModule(this.m_swerveSubsystem.m_frontLeftModuleState, this.m_swerveSubsystem.m_frontLeftModule);
      moveSwerveModule(this.m_swerveSubsystem.m_frontRightModuleState, this.m_swerveSubsystem.m_frontRightModule);
      moveSwerveModule(this.m_swerveSubsystem.m_backLeftModuleState, this.m_swerveSubsystem.m_backLeftModule);
      moveSwerveModule(this.m_swerveSubsystem.m_backRightModuleState, this.m_swerveSubsystem.m_backRightModule);
    } else {
      this.m_swerveSubsystem.m_backLeftModule.move(0, 0);
      this.m_swerveSubsystem.m_backRightModule.move(0, 0);
      this.m_swerveSubsystem.m_frontLeftModule.move(0, 0);
      this.m_swerveSubsystem.m_frontRightModule.move(0, 0);
    }
  }

  public void moveSwerveModule(SwerveModuleState swerveModuleState, SwerveModule swerveModule) {
    double driveSpeed = swerveModule.getDriveMotorSpeed(swerveModuleState.speedMetersPerSecond);
    double goalAngle = swerveModuleState.angle.getDegrees() + 180;
    SmartDashboard.putNumber(swerveModule.name + " swerve module state angle", goalAngle);
    SmartDashboard.putNumber(swerveModule.name + " turnmotor", swerveModule.getTurnMotorPosition());
    double ogDisplacement = goalAngle - swerveModule.getTurnMotorPosition();
    double displacement = SwerveUtil.getDisplacement(goalAngle, swerveModule.getTurnMotorPosition());
    // double displacement = getDisplacement(goalAngle, swerveModule.getTurnMotorPosition());
    if (!isWithinRange(swerveModule.getTurnMotorPosition(), goalAngle, 2.5)) {
      turnSpeed = swerveModule.getTurnMotorSpeed(displacement);
      swerveModule.move(Math.abs(ogDisplacement) > 90 && Math.abs(ogDisplacement) < 270 ? -driveSpeed : driveSpeed, turnSpeed);
    } else {
      turnSpeed = 0;
      swerveModule.move(driveSpeed, turnSpeed);
    }
    SmartDashboard.putNumber("displacement", displacement);
    SmartDashboard.putNumber(swerveModule.name + " turn speed", turnSpeed);  
  }

  public double getDisplacement(double goal, double initial) {
    double ogDisplacement = goal - initial;
    if (Math.abs(ogDisplacement) >= 270) {
      return (ogDisplacement) + (360 * Math.signum(initial - goal));
    } else if (Math.abs(ogDisplacement) > 90) {
      double bigD = (360 + (goal - 180)) % 360;
      return bigD - initial;
    } else {
      return (ogDisplacement);
    }
  }

  public Boolean isWithinRange(double value, double goal, double buffer) {
    return (value > goal - buffer && value < goal + buffer);
  }

  public double getGyroAngle(double gyroAngle) {
    return simplifyAngle(gyroAngle + 90);
  }

  public double simplifyAngle(double angle) {
    if (angle <= -180) {
      return angle + 360; 
    } else if (angle >= 180) {
      return angle - 360;
    }
    return angle;
  }

  public double getAngleAmount(double goalAngle, double currentAngle) {
    return simplifyAngle(goalAngle - currentAngle);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}