// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class TurnToAngle extends CommandBase {

  private PIDController controller = new PIDController(0.020, 0, .002);
  private Drivebase drive;
  private double relativeAngle;
  private Debouncer debouncer = new Debouncer(0.3);


  public TurnToAngle(Drivebase drive, double relativeAngle) {
    this.drive = drive;
    this.relativeAngle = relativeAngle;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    controller.setSetpoint(drive.getHeadingDegrees() + relativeAngle);
    controller.enableContinuousInput(-180, 180);
    controller.setTolerance(2);
  }

  @Override
  public void execute() {
    double output = controller.calculate(drive.getHeadingDegrees());
    drive.drive(MathUtil.clamp(output, -0.2, 0.2), -MathUtil.clamp(output, -.2, .2));
    SmartDashboard.putNumber("error", controller.getPositionError());
    SmartDashboard.putNumber("setpoinpt", controller.getSetpoint());
  }

  @Override
  public void end(boolean interrupted) {
    drive.drive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return debouncer.calculate(controller.atSetpoint());
  }
}
