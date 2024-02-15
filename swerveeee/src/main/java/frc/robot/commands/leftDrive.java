// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class leftDrive extends CommandBase {
  Drivebase m_subsystem;
  double speed;

  public leftDrive(
    Drivebase driveSubystem, 
    double speed
  ) {
    m_subsystem = driveSubystem;

    this.speed = speed;
    addRequirements(m_subsystem);
  }

  @Override
  public void execute() {
    
    m_subsystem. driveleft(speed);
    
  }
}
