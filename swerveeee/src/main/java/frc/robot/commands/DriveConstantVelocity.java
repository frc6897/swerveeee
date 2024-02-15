// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class DriveConstantVelocity extends CommandBase {

  private Drivebase driveSubsystem;
  private double speed;

  public DriveConstantVelocity(Drivebase driveSubsystem, double speed) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.speed = speed;
  }

  @Override
  public void execute() {
    driveSubsystem.curveDrive(speed, 0, false);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.curveDrive(0, 0, false);
  }

}
