// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {

  public static class DriveConstants {
    public static final double kMaxDriveSpeed = 0.2;
    public static final double kMaxTurnVoltage = 6;
    public static final double kDriveMotorGearRatio = 4.71;
    public static final double kTurnMotorGearRatio = 3;
    public static final double kWheelBase = Units.inchesToMeters(20);/* the distance between the front and rear wheels */
    public static final double kTrackWidth = Units.inchesToMeters(22.098); /* the distance between left and right wheels */
    public static final double kWheelDiameter = 5;

    public static final double kS = 0.15; // just under the minimum voltage needed to spin motor
    public static final double kV = 0.33; // velocity (RPM) / voltage

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
