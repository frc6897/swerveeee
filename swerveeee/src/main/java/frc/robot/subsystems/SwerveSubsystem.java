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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
  AHRS gyro = new AHRS(SPI.Port.kMXP);

  /**
   * One swerve module consists of both a drive and turn motor
   */
  public SwerveModule m_backLeftModule;
  public SwerveModule m_backRightModule;
  public SwerveModule m_frontLeftModule;
  public SwerveModule m_frontRightModule;

  /**
   * A swerve module state stores the speed and angle at which a wheel should be in order to reach a desired linear and angular speed
   * One swerve module state for each swerve module
   */
  public SwerveModuleState m_backLeftModuleState;
  public SwerveModuleState m_backRightModuleState;
  public SwerveModuleState m_frontLeftModuleState;
  public SwerveModuleState m_frontRightModuleState;

  /**
   * Store the location of each swerve module with respect to the center of the drivebase
   */  
  // Translation2d m_frontLeftLocation = new Translation2d(DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2);
  // Translation2d m_frontRightLocation = new Translation2d(DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2);
  // Translation2d m_backLeftLocation = new Translation2d(-DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2);
  // Translation2d m_backRightLocation = new Translation2d(-DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2);
  Translation2d m_frontLeftLocation = new Translation2d(-DriveConstants.kTrackWidth / 2, DriveConstants.kTrackWidth / 2);
  Translation2d m_frontRightLocation = new Translation2d(DriveConstants.kTrackWidth / 2, DriveConstants.kTrackWidth / 2);
  Translation2d m_backLeftLocation = new Translation2d(-DriveConstants.kTrackWidth / 2, -DriveConstants.kTrackWidth / 2);
  Translation2d m_backRightLocation = new Translation2d(DriveConstants.kTrackWidth / 2, -DriveConstants.kTrackWidth / 2);
  

  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  public SwerveSubsystem() {
    gyro.reset();
    this.m_frontLeftModule = new SwerveModule("Front Left: ", 2, 1, 270); 
    this.m_frontRightModule = new SwerveModule("Front Right: ", 4, 3, 0); 
    this.m_backLeftModule = new SwerveModule("Back Left: ", 8, 7, 180); 
    this.m_backRightModule = new SwerveModule("Back Right: ", 6, 5, 90); 
  }

  /**
   * Takes in BOTH joystick parameters, gets the desired speed and angle for each module, optimizes the states based on current robot angle
   * @param driveX left joystick x-axis (-1 to 1)
   * @param driveY left joystick y-axis (-1 to 1)
   * @param rotation rotating speed (-1 to 1)
   */
  public void setSwerveModuleStates(double driveX, double driveY, double rotation) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(driveY, -driveX, -rotation, Rotation2d.fromDegrees(getHeading()));
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(speeds);
    this.m_frontLeftModuleState = swerveModuleStates[0];
    this.m_frontRightModuleState = swerveModuleStates[1];
    this.m_backLeftModuleState = swerveModuleStates[2];
    this.m_backRightModuleState = swerveModuleStates[3];    
    // this.m_frontLeftModuleState = SwerveModuleState.optimize(swerveModuleStates[0], new Rotation2d(this.m_frontLeftModule.getTurnMotorPosition()));
    // this.m_frontRightModuleState = SwerveModuleState.optimize(swerveModuleStates[1], new Rotation2d(this.m_frontRightModule.getTurnMotorPosition()));
    // this.m_backLeftModuleState = SwerveModuleState.optimize(swerveModuleStates[2], new Rotation2d(this.m_backLeftModule.getTurnMotorPosition()));
    // this.m_backRightModuleState = SwerveModuleState.optimize(swerveModuleStates[3], new Rotation2d(this.m_backRightModule.getTurnMotorPosition()));
  }

  /**
   * Move the swerve module at the desired linear and angular speed
   * @param swerveModule swerve module
   * @param driveSpeed drive speed
   * @param angularSpeed angular speed
   */
  public void drive(SwerveModule swerveModule, double driveSpeed, double angularSpeed) {
    swerveModule.move(driveSpeed, angularSpeed);
  }

  public void setVoltage(SwerveModule swerve, double volts) {
    swerve.turnMotor.setVoltage(volts);
  }

  /**
   * Reset the gyro angle such that current robot angle is 0
   */
  public void resetGyro() {
    this.gyro.reset();
  }

  /**
   * Returns the current angle of the robot
   * @return current angle of the robot
   */
  public double getHeading() {
    return this.gyro.getYaw();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
