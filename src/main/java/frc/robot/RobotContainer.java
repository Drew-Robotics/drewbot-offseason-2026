// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.controller.DriverController;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.commands.DriveCommand;

public class RobotContainer {
  private final DriverController m_driverController = new DriverController (0); //MAKE THE GODDAMN CONSTANTS FILE FINNY - past you whose currently too lazy to

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    DriveSubsystem.getInstance().setDefaultCommand(
      new DriveCommand(
        m_driverController::getDriveX,
        m_driverController::getDriveY,
        m_driverController::getDriveRot
      )
    );
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
