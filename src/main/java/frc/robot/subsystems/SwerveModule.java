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
  public String name;

  public int angleCanId;
  public int driveCanId;
  
  public CANSparkMax turnMotor;
  public CANSparkMax driveMotor;
  
  public AbsoluteEncoder angleEncoder;
  public RelativeEncoder driveEncoder;

  public PIDController turnPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);

  public SimpleMotorFeedforward turnFF;
  public SimpleMotorFeedforward driveFF;

  public SlewRateLimiter driveSlewRateLimiter = new SlewRateLimiter(5.5);

  public double angleDisplacement;

  public double kS;
  public double kV;
  public double driveKS;
  public double driveKV;
  
  public SwerveModule(String name, int angleCanId, int driveCanId, double angleDisplacement, double kS, double kV, double dKS, double dKV) {
    this.name = name;

    driveKS = dKS;
    driveKV = dKV;

    this.kS = kS;
    this.kV = kV;

    this.angleCanId = angleCanId;
    this.driveCanId = driveCanId;

    this.angleDisplacement = angleDisplacement;

    turnFF = new SimpleMotorFeedforward(kS, kV);
    driveFF = new SimpleMotorFeedforward(driveKS, driveKV);

    turnMotor = new CANSparkMax(angleCanId, MotorType.kBrushless);

    driveMotor = new CANSparkMax(driveCanId, MotorType.kBrushless);

    angleEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle); // type?
    // turnMotor.restoreFactoryDefaults();
    angleEncoder.setPositionConversionFactor(360);

    driveEncoder = driveMotor.getEncoder();
    // driveMotor.restoreFactoryDekfaults();
    driveEncoder.setPositionConversionFactor(1/DriveConstants.driveGearRatio);

    configureTurnMotor();
    configureDriveMotor();

    turnPIDController.reset();

    angleEncoder.setVelocityConversionFactor(60);
    driveEncoder.setVelocityConversionFactor(1/(60 * DriveConstants.driveGearRatio));
  }

  /**
   * Sets default settings for the turn motor of the swerve module
   */
  public void configureTurnMotor() {
    turnMotor.setSmartCurrentLimit(60);
    turnMotor.setInverted(false);
    turnMotor.setIdleMode(IdleMode.kBrake);

    turnPIDController.enableContinuousInput(0, 360);
    turnPIDController.setTolerance(2.5);
  }

  /**
   * Sets default settings for the drive motor of the swerve module
   */
  public void configureDriveMotor() {
    driveMotor.setSmartCurrentLimit(60);
    driveMotor.setInverted(false);
    driveMotor.setIdleMode(IdleMode.kBrake);
  }

  /**
   * Return the encoder value of the drive motor (how far has the wheel traveled)
   * @return the drive encoder position (meters)
   */
  public double getDriveMotorPosition() {
    return driveEncoder.getPosition();
  }

  /**
   * Return the encoder value of the angle motor (how much has the wheel rotated)
   * @return the angle encoder position (0 to 360 degrees)
   */
  public double getTurnMotorPosition() {
    return Math.abs((angleEncoder.getPosition() + angleDisplacement) % 360);
  }

  /**
   * Set the motor speed of both the drive and turn motor
   * @param driveSpeed drive speed
   * @param turnSpeed turn speed
   */
  public void move(double driveVoltage, double turnVoltage) {
    driveMotor.setVoltage(MathUtil.clamp(driveVoltage, -DriveConstants.kMaxDriveVoltage, DriveConstants.kMaxDriveVoltage));
    turnMotor.setVoltage(MathUtil.clamp(turnVoltage, -DriveConstants.kMaxTurnVoltage, DriveConstants.kMaxTurnVoltage));
    // this.turnMotor.set(turnSpeed * DriveConstants.kMaxTurnSpeed);
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
    // This method will be called once per scheduler run
  }
}
