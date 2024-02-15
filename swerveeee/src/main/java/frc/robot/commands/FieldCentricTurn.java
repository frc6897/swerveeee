// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Drivebase;
import frc.robot.util.SwerveUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldCentricTurn extends CommandBase {
  Drivebase m_Drivebase;
  DoubleSupplier x;
  DoubleSupplier y;

  double m_controller_angle;

  public FieldCentricTurn(Drivebase drive, DoubleSupplier xAxis, DoubleSupplier yAxis) {
    m_Drivebase = drive;
    x = xAxis;
    y = yAxis;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    double tolerance = 2.5;

    double x_axis = x.getAsDouble();
    x_axis = Math.round(x_axis * 100);
    x_axis /= 100;

    double y_axis = y.getAsDouble() * -1;
    y_axis = Math.round(y_axis * 100);
    y_axis /= 100;

    double yaw = Math.round(this.m_Drivebase.getHeadingDegrees());

    this.m_controller_angle = SwerveUtil.getAngle(x_axis, y_axis, yaw);
 
    double speed = MathUtil.clamp((yaw - m_controller_angle) * 0.012, -0.5, 0.5);

    if ((yaw - m_controller_angle) >= 180 || (yaw - m_controller_angle) <= -180) {
      speed = -81 / (yaw - m_controller_angle);
    }

    if (Math.abs(yaw - m_controller_angle) < tolerance) {
      m_Drivebase.drive(0, 0);
    } else 
      m_Drivebase.curveDrive(0, speed, true);

    SmartDashboard.putNumber("controller", m_controller_angle);
  }
}
