// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SwerveModule extends SubsystemBase {
  
  public int angleCanId;
  public int driveCanId;
  
  public CANSparkMax angleMotor;
  public CANSparkMax driveMotor;
  
  public RelativeEncoder angleEncoder;
  public RelativeEncoder driveEncoder;
  
  /** Creates a new SwerveModule. */
  public SwerveModule(int angleCanId, int driveCanId) {
    this.angleCanId = angleCanId;
    this.driveCanId = driveCanId;
    
    this.angleMotor = new CANSparkMax(angleCanId, MotorType.kBrushless);
    this.angleMotor = new CANSparkMax(driveCanId, MotorType.kBrushless);
    
    this.angleEncoder = angleMotor.getEncoder();
    this.driveEncoder = driveMotor.getEncoder();
  }

  public void configureAngleMotor() {
    angleMotor.restoreFactoryDefaults();
    angleMotor.setSmartCurrentLimit(30);
    angleMotor.setInverted(false);
    angleMotor.setIdleMode(IdleMode.kBrake);
    angleMotor.burnFlash();
  }

  public void configureDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    driveMotor.setSmartCurrentLimit(30);
    driveMotor.setInverted(false);
    driveMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.burnFlash();
  }

  private final double MAX_VOLTS = 12; //guessed number

  public void driveWheel(double speed, double angle){
    driveMotor.set(speed);

    double setpoint = angle * (MAX_VOLTS * 0.5) + (MAX_VOLTS * 0.5);

    if(setpoint < 0){
      setpoint = MAX_VOLTS + setpoint;
    }
    if(setpoint > MAX_VOLTS){
      setpoint = setpoint - MAX_VOLTS;
    }

    
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
