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

  public void drive(double x1, double y1, double x2){
    //x1 and y1 are from the strafing joystick and x2 is from the rotation joystick
    
    // double hyp = Math.sqrt((L*L) + (W * W));

    // double a = x1 - x2 * (L / hyp);
    // double b = x1 + x2 * (L / hyp);
    // double c = y1 - x2 * (W / hyp);
    // double d = y1 + x2 * (W / hyp);

    // double backRightSpeed = Math.sqrt((a * a) + (d * d));
    // double backLeftSpeed = Math.sqrt((a * a) + (c * c));
    // double frontRightSpeed = Math.sqrt((b * b) + (d * d));
    // double frontLeftSpeed = Math.sqrt((b * b) + (c * c));

    // double backRightAngle = Math.atan2(a, d);
    // double backLeftAngle = Math.atan2(a, d);
    // double frontRightangle = Math.atan2(a, d);
    // double frontLeftAngle = Math.atan2(a, d);

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
