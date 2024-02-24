// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.DriveToDistance;
import frc.robot.commands.TurnVoltage;
import frc.robot.subsystems.*;

public class RobotContainer {

  PS4Controller driverGamepad = new PS4Controller(0);

  GenericHID macropad = new GenericHID(2);

  JoystickButton r1 = new JoystickButton(driverGamepad, PS4Controller.Button.kCircle.value);
  JoystickButton r2 = new JoystickButton(driverGamepad, PS4Controller.Button.kCross.value);

  JoystickButton change = new JoystickButton(macropad, 1);
  JoystickButton shootButton = new JoystickButton(macropad, 2);

  JoystickButton resetGyro = new JoystickButton(driverGamepad, PS4Controller.Button.kOptions.value);

  SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  // Drivebase m_drivebase = new Drivebase();


  public RobotContainer() {
    // m_swerveSubsystem.setDefaultCommand(new TestSwerve(this.m_swerveSubsystem, 2, 3, driverGamepad::getRightY, driverGamepad::getRightX));
    //change.whileTrue(new TeleopSwerve(this.m_swerveSubsystem, driverGamepad::getLeftX, driverGamepad::getLeftY, driverGamepad::getRightX, driverGamepad::getRightY, true));
    //change.whileFalse(new TeleopSwerve(this.m_swerveSubsystem, driverGamepad::getLeftX, driverGamepad::getLeftY, driverGamepad::getRightX, driverGamepad::getRightY, false));
    SmartDashboard.putBoolean("robot cont", true);

    m_swerveSubsystem.setDefaultCommand(new TeleopSwerve(this.m_swerveSubsystem, driverGamepad::getLeftX, driverGamepad::getLeftY, driverGamepad::getRightX, false));
    shootButton.whileTrue(new DriveToDistance(this.m_swerveSubsystem, 1, 0, -1));
    // m_shoot.setDefaultCommand(new TestShoot(m_shoot));
    // m_swerveSubsystem.setDefaultCommand(new TurnVoltage(m_swerveSubsystem));

    // m_drivebase.setDefaultCommand(
    //   new Drive(
    //     m_drivebase, 
    //     driverGamepad::getR2Axis,
    //     driverGamepad::getL2Axis,
    //     driverGamepad::getLeftX,
    //     driverGamepad::getRightX
    //   )
    // );
    
    
    // below was commented out 
    //  m_drivebase.setDefaultCommand(
    //    new DriveNormal(
    //      m_drivebase, 
    //      driverGamepad::getR2Axis,
    //      driverGamepad::getL2Axis,
    //      driverGamepad::getLeftX
    //    )
    //  );

  //  m_drivebase.setDefaultCommand(new FieldCentricTurn(m_drivebase, driverGamepad::getRightX, driverGamepad::getRightY));

    configureBindings();
  }

  private void configureBindings() {
    // resetGyro.onTrue(new ResetGyro(m_drivebase));
  }

  public Command getAutonomousCommand() {
    // return new AutoBalanceNew(m_drivebase);
    // return new DriveToDistance(m_drivebase, 1);
    // return new ParallelRaceGroup(new DriveConstantVelocity(m_drivebase, 0.5), new WaitCommand(2));
    // return new ShootTurnDrive(m_drivebase);
    // return new WaitCommand(5);
    return null;
  }
}
