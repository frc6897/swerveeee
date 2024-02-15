// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Drivebase extends SubsystemBase {
  AHRS gyro = new AHRS(SPI.Port.kMXP);
  CANSparkMax left1;
  CANSparkMax left2;
  CANSparkMax right1;
  CANSparkMax right2;

  SlewRateLimiter leftSlew = new SlewRateLimiter(1.2);
  SlewRateLimiter rightSlew = new SlewRateLimiter(1.2);

  public Drivebase() {
    left1 = new CANSparkMax(2, MotorType.kBrushless);
    left2 = new CANSparkMax (3, MotorType.kBrushless);
    right1 = new CANSparkMax(4, MotorType.kBrushless);
    right2 = new CANSparkMax (5, MotorType.kBrushless);
    left1.setIdleMode(IdleMode.kBrake);
    left2.setIdleMode(IdleMode.kBrake);
    left2.follow(left1);
    right1.setIdleMode(IdleMode.kBrake);
    right2.setIdleMode(IdleMode.kBrake);
    right2.follow(right1);
    right1.setInverted(false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("robot angle", getHeadingDegrees());
  }

  public void drive(double leftspeed, double rightspeed) {
    left1.set(leftspeed);
    right1.set(rightspeed);
  } 

  public void driveleft(double speed) {
    right1.set(speed);
  }

  public void curveDrive(double xSpeed, double rotation, boolean turn) {
    var speeds = DifferentialDrive.curvatureDriveIK(xSpeed, rotation, turn);
    drive(speeds.left, speeds.right);
  }

  public double getHeadingDegrees(){
    return gyro.getYaw();
  }

  public double getPitch() {
    return gyro.getRoll();
  }

  public double PIDSpeed(double position, double setpoint) {
    PIDController speedController = new PIDController(0.01, 0, 0);
    speedController.setTolerance(0.1);
    return speedController.calculate(position, setpoint);
  }
    
  public void resetGyro() {
    gyro.reset();
  }
}
