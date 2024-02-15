// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class Driving extends CommandBase {
  
  private Drivebase driveSubsystem;
  private DoubleSupplier fAxis;
  private DoubleSupplier bAxis;

  public Driving(Drivebase driveSubsystem, DoubleSupplier fAxis, DoubleSupplier bAxis) {
    this.driveSubsystem = driveSubsystem;
    this.fAxis = fAxis;
    this.bAxis = bAxis;
    addRequirements(driveSubsystem);
  }
  
  @Override
  public void execute() {
    double speed = (fAxis.getAsDouble() - bAxis.getAsDouble()) * 0.5;
    driveSubsystem.drive(speed, speed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
