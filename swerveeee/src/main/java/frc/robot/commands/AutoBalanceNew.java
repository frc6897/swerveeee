// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivebase;

public class AutoBalanceNew extends SequentialCommandGroup {
  public AutoBalanceNew(Drivebase driveSubsystem) {
    addCommands(
      new DriveConstantVelocity(driveSubsystem, 0.2).until(() -> driveSubsystem.getPitch() > 5),
      new AutoBalanceDrive(driveSubsystem)
    );
  }
}
