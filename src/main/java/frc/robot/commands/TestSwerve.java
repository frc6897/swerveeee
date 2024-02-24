// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;

public class TestSwerve extends Command {
  public SwerveModule m_swerveModule;
  public SwerveSubsystem m_swerveSubsystem;
  DoubleSupplier drive;
  DoubleSupplier turn;

  /** Creates a new TestSwerve. */
  public TestSwerve(SwerveSubsystem swerveSubsystem, int angleId, int driveId, DoubleSupplier drive, DoubleSupplier turn) {
    // this.m_swerveModule = new SwerveModule(angleId, driveId);
    this.m_swerveSubsystem = swerveSubsystem;
    this.drive = drive;
    this.turn = turn;
    this.addRequirements(swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putString("DSAD", "asdasdsad");
    // double driveSpeed = drive.getAsDouble() < 0.1 ? 0 : drive.getAsDouble();
    // double turnSpeed = turn.getAsDouble() < 0.1 ? 0 : turn.getAsDouble();
    // this.m_swerveModule.move(driveSpeed, turnSpeed);
    this.m_swerveModule.move(drive.getAsDouble(), turn.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.m_swerveModule.move(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
