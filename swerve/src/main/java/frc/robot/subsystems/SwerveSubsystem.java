// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.*;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  SwerveModule bottomLeftWheel;
  SwerveModule bottomRightWheel;
  SwerveModule topLeftWheel;
  SwerveModule topRightWheel;



  public SwerveSubsystem() {
    bottomLeftWheel = new SwerveModule(0, 0);
    bottomRightWheel = new SwerveModule(0, 0);
    topLeftWheel = new SwerveModule(0, 0);
    topRightWheel = new SwerveModule(0, 0);
  }

  public final double L = 1;
  public final double W = 1;

  public void drive(int driveX, int driveY, int rotationX, int rotationY){

  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
