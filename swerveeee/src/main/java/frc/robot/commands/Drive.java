// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class Drive extends CommandBase {
  Drivebase m_subsystem;
  DoubleSupplier m_forwardsSupplier;
  DoubleSupplier m_backwardsSupplier;
  DoubleSupplier m_curveSupplier;
  DoubleSupplier m_turnSupplier;

  public Drive(
    Drivebase driveSubystem, 
    DoubleSupplier forwardsSupplier,
    DoubleSupplier backwardsSupplier,
    DoubleSupplier curveAxisSupplier,
    DoubleSupplier turnInPlaceAxisSupplier
  ) {
    m_subsystem = driveSubystem;
    m_forwardsSupplier = forwardsSupplier;
    m_backwardsSupplier = backwardsSupplier;
    m_curveSupplier = curveAxisSupplier;
    m_turnSupplier = turnInPlaceAxisSupplier;

    addRequirements(m_subsystem);
  }

  @Override
  public void execute() {
    double speedMultiplier = 1;
    double turnMultiplier = .8;

    double speed = (m_forwardsSupplier.getAsDouble() - m_backwardsSupplier.getAsDouble()) * speedMultiplier;

    if (Math.abs(m_turnSupplier.getAsDouble()) >= .1) {
      m_subsystem.curveDrive(0, -m_turnSupplier.getAsDouble() * turnMultiplier, true);
    } else {
      m_subsystem.curveDrive(speed, -m_curveSupplier.getAsDouble(), false);
    }
  }
}
