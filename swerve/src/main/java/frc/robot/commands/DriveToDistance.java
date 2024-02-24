// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Date;

import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToDistance extends Command {
  private SwerveSubsystem m_swerveSubsystem;
  private double m_distanceToTravel;
  private double m_speedX;
  private double m_speedY;

  public DriveToDistance(SwerveSubsystem swerveSubsystem, double distanceToTravel, double speedX, double speedY) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.m_distanceToTravel = distanceToTravel;
    this.m_speedX = speedX;
    this.m_speedY = speedY;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    this.m_swerveSubsystem.resetEncoders();
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("Drive Encoder Position: ", this.m_swerveSubsystem.getEncoderPosition());
    this.m_swerveSubsystem.drive(m_speedX, m_speedY, 0);
  }

  @Override
  public void end(boolean interrupted) {
    this.m_swerveSubsystem.drive(0, 0, 0);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(this.m_swerveSubsystem.getEncoderPosition()) > Math.abs(this.m_distanceToTravel);
  }
}
