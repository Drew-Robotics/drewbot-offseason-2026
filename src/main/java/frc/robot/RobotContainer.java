// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.controller.DriverController;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.commands.DriveCommand;
import frc.robot.constants.MiscConstants;

public class RobotContainer {
  private final DriverController m_driverController;
  private final CommandXboxController m_characterizationController;
  private final DriveSubsystem m_driveSub;
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  private final SendableChooser<SysIdMechanism> m_sysIdChooser = new SendableChooser<>();

  public RobotContainer() {
    m_driverController = new DriverController(MiscConstants.kDriverControllerPort);
    m_characterizationController = new CommandXboxController(MiscConstants.kCharacterizationControllerPort);
    m_driveSub = new DriveSubsystem();
    configureBindings();
    configureAutos();
  }

  private void configureBindings() {
    m_driveSub.setDefaultCommand(
      new DriveCommand(
        m_driveSub,
        m_driverController::getDriveX,
        m_driverController::getDriveY,
        m_driverController::getDriveRot
      )
    );

    m_driverController.start().onTrue(Commands.runOnce(m_driveSub::zeroHeading, m_driveSub).ignoringDisable(true));
    m_driverController.x().whileTrue(Commands.run(m_driveSub::xLock, m_driveSub));

    configureCharacterizationBindings();
  }

  private enum SysIdMechanism { DRIVE, TURN, ROTATION }

  /**
   * Second controller, teleop. Pick the routine in "SysId Routine" on the dashboard, then HOLD:
   * A = quasistatic forward, B = quasistatic reverse, X = dynamic forward, Y = dynamic reverse.
   * Letting go stops the test. Restart robot code between mechanisms so each gets its own log.
   */
  private void configureCharacterizationBindings() {
    m_sysIdChooser.setDefaultOption("Drive", SysIdMechanism.DRIVE);
    m_sysIdChooser.addOption("Turn (one module)", SysIdMechanism.TURN);
    m_sysIdChooser.addOption("Rotation (heading PID)", SysIdMechanism.ROTATION);
    SmartDashboard.putData("SysId Routine", m_sysIdChooser);

    m_characterizationController.a().whileTrue(Commands.select(Map.of(
      SysIdMechanism.DRIVE, m_driveSub.sysIdDriveQuasistatic(Direction.kForward),
      SysIdMechanism.TURN, m_driveSub.sysIdTurnQuasistatic(Direction.kForward),
      SysIdMechanism.ROTATION, m_driveSub.sysIdRotationQuasistatic(Direction.kForward)
    ), m_sysIdChooser::getSelected));
    m_characterizationController.b().whileTrue(Commands.select(Map.of(
      SysIdMechanism.DRIVE, m_driveSub.sysIdDriveQuasistatic(Direction.kReverse),
      SysIdMechanism.TURN, m_driveSub.sysIdTurnQuasistatic(Direction.kReverse),
      SysIdMechanism.ROTATION, m_driveSub.sysIdRotationQuasistatic(Direction.kReverse)
    ), m_sysIdChooser::getSelected));
    m_characterizationController.x().whileTrue(Commands.select(Map.of(
      SysIdMechanism.DRIVE, m_driveSub.sysIdDriveDynamic(Direction.kForward),
      SysIdMechanism.TURN, m_driveSub.sysIdTurnDynamic(Direction.kForward),
      SysIdMechanism.ROTATION, m_driveSub.sysIdRotationDynamic(Direction.kForward)
    ), m_sysIdChooser::getSelected));
    m_characterizationController.y().whileTrue(Commands.select(Map.of(
      SysIdMechanism.DRIVE, m_driveSub.sysIdDriveDynamic(Direction.kReverse),
      SysIdMechanism.TURN, m_driveSub.sysIdTurnDynamic(Direction.kReverse),
      SysIdMechanism.ROTATION, m_driveSub.sysIdRotationDynamic(Direction.kReverse)
    ), m_sysIdChooser::getSelected));

    //hold right bumper with the front bumper against a wall, let go to stop
    m_characterizationController.rightBumper().whileTrue(m_driveSub.slipCurrentCharacterization());
  }

  private void configureAutos() {
    m_autoChooser.setDefaultOption("None", Commands.none());
    m_autoChooser.addOption("Wheel Radius Characterization", m_driveSub.wheelRadiusCharacterization());

    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
