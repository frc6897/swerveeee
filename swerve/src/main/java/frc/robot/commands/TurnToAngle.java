// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;

public class TurnToAngle extends Command {
  private SwerveSubsystem m_swerveSubsystem;
  private double m_angle;
  private PIDController controller = new PIDController(1, 0, 0);

  public TurnToAngle(SwerveSubsystem swerveSubsystem, double angle, double rotationSpeed) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.m_angle = angle;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    controller.setSetpoint(this.m_swerveSubsystem.getHeading() + this.m_angle);
    controller.enableContinuousInput(-180, 180);
    controller.setTolerance(1.5);
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("Robot Angle: ", this.m_swerveSubsystem.getHeading());
    double calculatedRotationSpeed = controller.calculate(this.m_swerveSubsystem.getHeading());
    this.m_swerveSubsystem.drive(0, 0, calculatedRotationSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    this.m_swerveSubsystem.drive(0, 0, 0);
  }

  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
