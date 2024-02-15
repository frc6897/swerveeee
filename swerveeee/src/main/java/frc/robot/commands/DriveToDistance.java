/********************************************************************************
*                                                                               *
*   Copyright (c) Astraea Robotics, FIRST, and other WPILib contributors        *
*                                                                               *
*   Open Source Software; you can modify and/or share it under the terms of     *
*   the license file in the root directory of this project.                     *
*                                                                               *
********************************************************************************/
package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class DriveToDistance extends CommandBase {

  private final Drivebase m_driveSubsystem;
  private double distanceMeters;

  public DriveToDistance(Drivebase driveSubsystem, double distanceMeters) {
    this.m_driveSubsystem = driveSubsystem;
    this.distanceMeters = distanceMeters;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // double speed = MathUtil.clamp(m_driveSubsystem.PIDSpeed(m_driveSubsystem.getEncoderPosition(), distanceMeters), 0.1, 0.4);
    double speed = 0.2;
    m_driveSubsystem.drive(speed, speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
    // return Math.abs(m_driveSubsystem.getEncoderPosition()) > Math.abs(distanceMeters);
  }
}