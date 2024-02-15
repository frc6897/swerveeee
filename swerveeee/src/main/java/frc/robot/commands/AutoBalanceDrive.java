// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class AutoBalanceDrive extends CommandBase {
  
  public Drivebase driveSubsystem;
  public double speed;

  public AutoBalanceDrive(Drivebase driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    this.speed = MathUtil.clamp(this.driveSubsystem.getPitch() * 0.017, -0.2, 0.2); // kyle's made up math determined the number
    this.driveSubsystem.curveDrive(speed, 0, false);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(driveSubsystem.getPitch()) < 5;
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.curveDrive(0, 0, false);
  }
}
