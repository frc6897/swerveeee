// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class DriveNormal extends CommandBase {
  /** Creates a new DriveNormal. */

  private Drivebase driveSub;
  private DoubleSupplier turnAxis;
  private DoubleSupplier fAxis;
  private DoubleSupplier bAxis;



  public DriveNormal(Drivebase drivesub, DoubleSupplier fwdSupplier, DoubleSupplier bwdSupplier, DoubleSupplier turnSupplier) {

    driveSub = drivesub;
    turnAxis = turnSupplier;
    fAxis = fwdSupplier;
    bAxis = bwdSupplier;
    addRequirements(drivesub);

  }

  @Override
  public void execute() {
    driveSub.drive((fAxis.getAsDouble() - bAxis.getAsDouble() + (turnAxis.getAsDouble() * -1)) * 0.4, (fAxis.getAsDouble() - bAxis.getAsDouble() - (turnAxis.getAsDouble() * -1)) * 0.4);
  }
}
