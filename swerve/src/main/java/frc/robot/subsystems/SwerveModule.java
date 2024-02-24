// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveModule extends SubsystemBase {
  public String m_name;

  public int m_angleCanId;
  public int m_driveCanId;
  
  public CANSparkMax turnMotor;
  public CANSparkMax driveMotor;
  
  public AbsoluteEncoder angleEncoder;
  public RelativeEncoder driveEncoder;

  public PIDController turnPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);

  public SimpleMotorFeedforward turnFF;
  public SimpleMotorFeedforward driveFF;

  public SlewRateLimiter driveSlewRateLimiter = new SlewRateLimiter(5.5);

  public double m_angleDisplacement;

  public double m_kS;
  public double m_kV;
  public double m_driveKS;
  public double m_driveKV;
  
  public SwerveModule(String name, int angleCanId, int driveCanId, double angleDisplacement, double kS, double kV, double dKS, double dKV) {
    this.m_name = name;

    this.m_driveKS = dKS;
    this.m_driveKV = dKV;

    this.m_kS = kS;
    this.m_kV = kV;

    this.m_angleCanId = angleCanId;
    this.m_driveCanId = driveCanId;

    this.m_angleDisplacement = angleDisplacement;

    turnFF = new SimpleMotorFeedforward(kS, kV);
    driveFF = new SimpleMotorFeedforward(this.m_driveKS, this.m_driveKV);

    configureTurnMotor();
    configureDriveMotor();
  }

  public void configureTurnMotor() {
    turnMotor = new CANSparkMax(this.m_angleCanId, MotorType.kBrushless);

    angleEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
    angleEncoder.setPositionConversionFactor(360);
    angleEncoder.setVelocityConversionFactor(60);

    turnMotor.setSmartCurrentLimit(60);
    turnMotor.setInverted(false);
    turnMotor.setIdleMode(IdleMode.kBrake);

    turnPIDController.enableContinuousInput(0, 360);
    turnPIDController.setTolerance(2.5);
    turnPIDController.reset();
  }

  public void configureDriveMotor() {
    driveMotor = new CANSparkMax(this.m_driveCanId, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    driveEncoder.setPositionConversionFactor((1 / DriveConstants.driveGearRatio) * 2 * Math.PI * (DriveConstants.kWheelDiameter / 2));
    driveEncoder.setVelocityConversionFactor(1/(60 * DriveConstants.driveGearRatio));

    driveMotor.setSmartCurrentLimit(60);
    driveMotor.setInverted(false);
    driveMotor.setIdleMode(IdleMode.kBrake);
  }

  public double getDriveMotorPosition() {
    return driveEncoder.getPosition();
  }

  public double getTurnMotorPosition() {
    return Math.abs((angleEncoder.getPosition() + this.m_angleDisplacement) % 360);
  }

  public void move(double driveVoltage, double turnVoltage) {
    driveMotor.setVoltage(MathUtil.clamp(driveVoltage, -DriveConstants.kMaxDriveVoltage, DriveConstants.kMaxDriveVoltage));
    turnMotor.setVoltage(MathUtil.clamp(turnVoltage, -DriveConstants.kMaxTurnVoltage, DriveConstants.kMaxTurnVoltage));
  }

  public double getDriveMotorSpeed(double speed) {
    SmartDashboard.putNumber("Target Voltage", driveFF.calculate(speed));
    return driveSlewRateLimiter.calculate(driveFF.calculate(speed));
  }

  public double getTurnMotorSpeed(double displacement) {
    return turnFF.calculate(turnPIDController.calculate(displacement, 0)); 
  }

  public double getTurnVelocity() {
    return angleEncoder.getVelocity();
  }

  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  @Override
  public void periodic() {
  }
}
