// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SwerveUtil;


public class TeleopSwerve extends CommandBase {

  SwerveSubsystem m_swerveSubsystem;

  DoubleSupplier m_driveX;
  DoubleSupplier m_driveY;
  DoubleSupplier m_rotationX;
  DoubleSupplier m_rotationY;

  Boolean press;

  
  public TeleopSwerve(SwerveSubsystem swerveSubsystem, DoubleSupplier driveX, DoubleSupplier driveY, DoubleSupplier rotationX, DoubleSupplier rotationY, Boolean press) {
    m_swerveSubsystem = swerveSubsystem;
    addRequirements(m_swerveSubsystem);
    m_driveX = driveX;
    m_driveY = driveY;
    m_rotationX = rotationX;
    m_rotationY = rotationY;
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
    double rotationAngle;
    if (Math.abs(this.m_rotationX.getAsDouble()) >= .1 || Math.abs(this.m_rotationY.getAsDouble()) >= .1) 
      rotationAngle = -SwerveUtil.getAngle(this.m_rotationX.getAsDouble(), this.m_rotationY.getAsDouble(), m_swerveSubsystem.getHeading()) + 180;
    else {
      rotationAngle = 0;
    }

    double rotationSpeed = this.m_rotationX.getAsDouble();
    SmartDashboard.putNumber("robot angle", this.m_swerveSubsystem.getHeading());
    SmartDashboard.putNumber("velocity", m_swerveSubsystem.m_frontRightModule.getVelocity());

      // this.m_swerveSubsystem.m_backLeftModule.move(0.05, 0.05);
      // this.m_swerveSubsystem.m_backRightModule.move(0.05, 0.05);
      // this.m_swerveSubsystem.m_frontLeftModule.move(0.05, 0.05);
      // this.m_swerveSubsystem.m_frontRightModule.move(0.05, 0.05);

    if (this.press == true) {
      m_swerveSubsystem.setSwerveModuleStates(0.1, 0, 0);
    } else {
      m_swerveSubsystem.setSwerveModuleStates(0, 0.1, 0);
    }
    moveSwerveModule(m_swerveSubsystem.m_backLeftModuleState, m_swerveSubsystem.m_backLeftModule);
    moveSwerveModule(m_swerveSubsystem.m_backRightModuleState, m_swerveSubsystem.m_backRightModule);
    moveSwerveModule(m_swerveSubsystem.m_frontLeftModuleState, m_swerveSubsystem.m_frontLeftModule);
    moveSwerveModule(m_swerveSubsystem.m_frontRightModuleState, m_swerveSubsystem.m_frontRightModule);
            
      
      
    // if (Math.abs(this.m_driveX.getAsDouble()) >= 0.1 || Math.abs(this.m_driveY.getAsDouble()) >= 0.1 || Math.abs(this.m_rotationX.getAsDouble()) >= .1 || Math.abs(this.m_rotationY.getAsDouble()) >= .1) {
    //   this.m_swerveSubsystem.setSwerveModuleStates(this.m_driveX.getAsDouble(), this.m_driveY.getAsDouble(), rotationSpeed);
    //   moveSwerveModule(this.m_swerveSubsystem.m_backLeftModuleState, this.m_swerveSubsystem.m_backLeftModule);
    //   moveSwerveModule(this.m_swerveSubsystem.m_backRightModuleState, this.m_swerveSubsystem.m_backRightModule);
    //   moveSwerveModule(this.m_swerveSubsystem.m_frontLeftModuleState, this.m_swerveSubsystem.m_frontLeftModule);
    //   moveSwerveModule(this.m_swerveSubsystem.m_frontRightModuleState, this.m_swerveSubsystem.m_frontRightModule);
    // } else {
    //   this.m_swerveSubsystem.m_backLeftModule.move(0, 0);
    //   this.m_swerveSubsystem.m_backRightModule.move(0, 0);
    //   this.m_swerveSubsystem.m_frontLeftModule.move(0, 0);
    //   this.m_swerveSubsystem.m_frontRightModule.move(0, 0);
    // }
  }

  public void moveSwerveModule(SwerveModuleState state, SwerveModule mod) {
    double driveSpeed = mod.getDriveMotorSpeed(state.speedMetersPerSecond);
    double goalAngle = state.angle.getDegrees() + 180;
    double ogD = goalAngle - mod.getTurnMotorPosition();
    double d = SwerveUtil.getDisplacement(goalAngle, mod.getTurnMotorPosition());
    if (!(Math.abs(goalAngle - mod.getTurnMotorPosition()) < 2.5)) {
      double turnSpeed = mod.getTurnMotorSpeed(d);
      double newDriveSpeed = Math.abs(ogD) > 90 && Math.abs(ogD) < 270 ? -driveSpeed : driveSpeed;
      mod.move(newDriveSpeed, turnSpeed);
    } else {
      double turnSpeed = 0;
      mod.move(driveSpeed, turnSpeed);
    }
  }
}
