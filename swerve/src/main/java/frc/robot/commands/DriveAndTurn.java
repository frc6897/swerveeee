// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Date;

import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveAndTurn extends Command {
  private SwerveSubsystem m_swerveSubsystem;
  private double m_distanceToTravel;
  private double m_angle;
  private double m_speedX;
  private double m_speedY;

  public DriveAndTurn(SwerveSubsystem swerveSubsystem, double distanceToTravel, double angle, double speedX, double speedY) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.m_distanceToTravel = distanceToTravel;
    this.m_angle = angle;
    this.m_speedX = speedX;
    this.m_speedY = speedY;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    // double time = 
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
