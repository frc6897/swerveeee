// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.DriveConstants;
import frc.robot.subsystems.*;

public class SwerveSubsystem extends SubsystemBase {
  AHRS gyro = new AHRS(SPI.Port.kMXP);

  SwerveModule bottomLeftModule;
  SwerveModule bottomRightModule;
  SwerveModule topLeftModule;
  SwerveModule topRightModule;

  Translation2d topLeftLocation = new Translation2d(DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2);
  Translation2d topRightLocation = new Translation2d(DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2);
  Translation2d bottomLeftLocation = new Translation2d(-DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2);
  Translation2d bottomRightLocation = new Translation2d(-DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2);

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(topLeftLocation, topRightLocation, bottomLeftLocation, bottomRightLocation);
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(getHeading()));

  public SwerveSubsystem() {
    bottomLeftModule = new SwerveModule(0, 0);
    bottomRightModule = new SwerveModule(0, 0);
    topLeftModule = new SwerveModule(0, 0);
    topRightModule = new SwerveModule(0, 0);
  }

  public SwerveModuleStates getSwerveModuleStates(int driveX, int driveY, int rotation) {
    ChassisSpeeds speeds;
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(driveY, driveX, rotation, Rotation2d.fromDegrees(getHeading()));
    return kinematics.toSwerveModuleStates(speeds);
  }

  public void drive(SwerveModuleStates swerveModuleStates) {
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, DriveConstants.kMaxDriveSpeed);
    topLeftModule.move(states[0].speedMetersPerSecond / DriveConstants.kMaxDriveSpeed, states[0].angle.getRadians() / (2 * Math.PI));
    topRightModule.move(states[1].speedMetersPerSecond / DriveConstants.kMaxDriveSpeed, states[1].angle.getRadians() / (2 * Math.PI));
    bottomLeftModule.move(states[2].speedMetersPerSecond / DriveConstants.kMaxDriveSpeed, states[2].angle.getRadians() / (2 * Math.PI));
    bottomRightModule.move(states[3].speedMetersPerSecond / DriveConstants.kMaxDriveSpeed, states[3].angle.getRadians() / (2 * Math.PI));
  }

  public double getHeading() {
    return gyro.getYaw();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
