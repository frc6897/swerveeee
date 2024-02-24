// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.*;

public class TurnVoltage extends Command {
  /** Creates a new TurnVoltage. */

  SwerveModule swerveMod;
  SwerveSubsystem m_swerve;

  public TurnVoltage(SwerveSubsystem m_swerve) {
    addRequirements(m_swerve);
    this.m_swerve = m_swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerve.setVoltage(m_swerve.m_frontLeftModule, 5);

    SmartDashboard.putNumber("Module Velocity", m_swerve.m_frontLeftModule.getDriveVelocity());
  }

}
