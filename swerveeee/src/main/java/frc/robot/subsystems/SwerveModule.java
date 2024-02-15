// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveConstantVelocity;

public class SwerveModule extends SubsystemBase {
  public String name;

  public int angleCanId;
  public int driveCanId;
  
  public CANSparkMax turnMotor;
  public CANSparkMax driveMotor;
  
  public AbsoluteEncoder angleEncoder;
  public RelativeEncoder driveEncoder;

  public PIDController turnPIDController = new PIDController(0.01, 0, 0);

  public SimpleMotorFeedforward turnFF = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV);

  public SlewRateLimiter driveSlewRateLimiter = new SlewRateLimiter(3);

  public double angleDisplacement;
  
  public SwerveModule(String name, int angleCanId, int driveCanId, double angleDisplacement) {
    this.name = name;

    this.angleCanId = angleCanId;
    this.driveCanId = driveCanId;

    this.angleDisplacement = angleDisplacement;

    this.turnMotor = new CANSparkMax(angleCanId, MotorType.kBrushless);
    // set the conversion for angle motor

    this.driveMotor = new CANSparkMax(driveCanId, MotorType.kBrushless);

    this.angleEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle); // type?
    this.angleEncoder.setPositionConversionFactor(360);

    this.driveEncoder = driveMotor.getEncoder();

    this.configureTurnMotor();
    this.configureDriveMotor();

    this.turnPIDController.reset();

    angleEncoder.setVelocityConversionFactor(60);
  }

  /**
   * Sets default settings for the turn motor of the swerve module
   */
  public void configureTurnMotor() {
    turnMotor.setSmartCurrentLimit(60);
    turnMotor.setInverted(false);
    turnMotor.setIdleMode(IdleMode.kBrake);
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
  public void move(double driveSpeed, double turnVoltage) {
    this.driveMotor.set(MathUtil.clamp(driveSpeed, -DriveConstants.kMaxDriveSpeed, DriveConstants.kMaxDriveSpeed));
    this.turnMotor.setVoltage(MathUtil.clamp(turnVoltage, -DriveConstants.kMaxTurnVoltage, DriveConstants.kMaxTurnVoltage));
    // this.turnMotor.set(turnSpeed * DriveConstants.kMaxTurnSpeed);
  }

  public double getDriveMotorSpeed(double speed) {
    return driveSlewRateLimiter.calculate(speed);
  }

  public double getTurnMotorSpeed(double displacement) {
    return turnFF.calculate(turnPIDController.calculate(displacement, 0)); 
  }

  public double getVelocity() {
    return angleEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
