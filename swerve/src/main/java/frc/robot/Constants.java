// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {

  public static class DriveConstants {
    public static final double kMaxDriveVoltage = 12;
    public static final double kMaxTurnVoltage = 11;
    public static final double kDriveMotorGearRatio = 4.71;
    public static final double kTurnMotorGearRatio = 3;
    public static final double kWheelBase = Units.inchesToMeters(20);/* the distance between the front and rear wheels */
    public static final double kTrackWidth = Units.inchesToMeters(22); /* the distance between left and right wheels */
    public static final double kWheelDiameter = Units.inchesToMeters(3);

    public static final double driveVelConvFactor = 1 / (0.0381 * 2 * Math.PI);
    public static final double driveGearRatio = 3.56;

    // GOOD PID VALUES BUT FOR SLOW TURN
    public static final double kP = 0.125; 
    public static final double kI = 0.0001;
    public static final double kD = 0;
    
    // TURN FF VALUES:

    // Front Right Module
    public static final double fRkS = 0.15; // just under the minimum voltage needed to spin motor
    public static final double fRkV = 0.323; // velocity (RPS) / voltage

    // Front Left Module
    public static final double fLkS = 0.15;
    public static final double fLkV = 0.311;

    // Back Left Module
    public static final double bLkS = 0.18;
    public static final double bLkV = 0.297;

    // Back Right Module
    public static final double bRkS = 0.16;
    public static final double bRkV = 0.311;

    // DRIVE FF VALUES:

    // Front Right Module
    public static final double drivefRkS = 0.1;
    public static final double drivefRkV = 5 / 11.37;

    // Front Left Module
    public static final double drivefLkS = 0.12;
    public static final double drivefLkV = 5 / 11.05;

    // Back Right Module
    public static final double drivebRkS = 0.11;
    public static final double drivebRkV = 5 / 10.9;

    // Back Left Module
    public static final double drivebLkS = 0.11;
    public static final double drivebLkV = 5 / 11.58;

  }

  public static class ShooterConstants {

  }

  public static class IntakeConstants {

  }

  public static class ElevatorConstants {

  }

  public static class HoodConstants {
    
  }
}
