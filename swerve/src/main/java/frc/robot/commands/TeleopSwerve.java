// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class TeleopSwerve extends CommandBase {
  public SwerveSubsystem swerveSubsystem;
  public int driveX;
  public int driveY;
  public int rotationX;
  public int rotationY;

  /** Creates a new TeleopSwerve. */
  public TeleopSwerve(SwerveSubsystem swerveSubsystem, int driveX, int driveY, int rotationX, int rotationY) {
    this.swerveSubsystem = swerveSubsystem;
    this.driveX = driveX;
    this.driveY = driveY;
    this.rotationX = rotationX;
    this.rotationY = rotationY;
    addRequirements();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.swerveSubsystem.drive(this.driveX, this.driveY, this.rotationX, this.rotationY);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
