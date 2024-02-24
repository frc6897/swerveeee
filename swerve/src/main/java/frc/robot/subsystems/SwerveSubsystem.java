// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.SwerveUtil;

public class SwerveSubsystem extends SubsystemBase {
  AHRS gyro = new AHRS(SPI.Port.kMXP);

  public SwerveModule m_backLeftModule;
  public SwerveModule m_backRightModule;
  public SwerveModule m_frontLeftModule;
  public SwerveModule m_frontRightModule;

  public SwerveModuleState m_backLeftModuleState;
  public SwerveModuleState m_backRightModuleState;
  public SwerveModuleState m_frontLeftModuleState;
  public SwerveModuleState m_frontRightModuleState;

  Translation2d m_frontLeftLocation = new Translation2d(DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2);
  Translation2d m_frontRightLocation = new Translation2d(DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2);
  Translation2d m_backLeftLocation = new Translation2d(-DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2);
  Translation2d m_backRightLocation = new Translation2d(-DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2);  

  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  public SwerveSubsystem() {
    gyro.reset();
    this.m_frontLeftModule = new SwerveModule("Front Left: ", 2, 1, 270, DriveConstants.fLkS, DriveConstants.fLkV, DriveConstants.drivefLkS, DriveConstants.drivefLkV); 
    this.m_frontRightModule = new SwerveModule("Front Right: ", 4, 3, 0, DriveConstants.fRkS, DriveConstants.fRkV, DriveConstants.drivefRkS, DriveConstants.drivefRkV); 
    this.m_backLeftModule = new SwerveModule("Back Left: ", 8, 7, 180, DriveConstants.bLkS, DriveConstants.bLkV, DriveConstants.drivebLkS, DriveConstants.drivebLkV); 
    this.m_backRightModule = new SwerveModule("Back Right: ", 6, 5, 90, DriveConstants.bRkS, DriveConstants.bRkV, DriveConstants.drivebRkS, DriveConstants.drivebRkV); 
  }

  public void setSwerveModuleStates(double driveX, double driveY, double rotation) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(driveY, -driveX, -rotation, Rotation2d.fromDegrees(getHeading()));
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(speeds);
    this.m_frontLeftModuleState = swerveModuleStates[0];
    this.m_frontRightModuleState = swerveModuleStates[1];
    this.m_backLeftModuleState = swerveModuleStates[2];
    this.m_backRightModuleState = swerveModuleStates[3];    
  }

  public void setVoltage(SwerveModule swerve, double volts) {
    swerve.driveMotor.setVoltage(volts);
  }

  public void resetGyro() {
    this.gyro.reset();
  }

  public double getHeading() {
    return this.gyro.getYaw();
  }

  public void resetEncoders() {
    this.m_frontLeftModule.driveEncoder.setPosition(0);
    this.m_frontRightModule.driveEncoder.setPosition(0);
    this.m_backLeftModule.driveEncoder.setPosition(0);
    this.m_backRightModule.driveEncoder.setPosition(0);
  }

  public double getEncoderPosition() {
    return this.m_frontLeftModule.driveEncoder.getPosition();
  }

  public void drive(double speedX, double speedY, double rotationSpeed) {
    this.setSwerveModuleStates(speedX, speedY, rotationSpeed);
    this.moveAllSwerveModules();
  }

  public void moveAllSwerveModules() {
    SwerveUtil.moveSwerveModule(this.m_backLeftModuleState, this.m_backLeftModule);
    SwerveUtil.moveSwerveModule(this.m_backRightModuleState, this.m_backRightModule);
    SwerveUtil.moveSwerveModule(this.m_frontLeftModuleState, this.m_frontLeftModule);
    SwerveUtil.moveSwerveModule(this.m_frontRightModuleState, this.m_frontRightModule);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
