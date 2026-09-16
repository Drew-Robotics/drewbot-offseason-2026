// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.DriveConstants.BackLeftModule;
import frc.robot.constants.DriveConstants.BackRightModule;
import frc.robot.constants.DriveConstants.FrontLeftModule;
import frc.robot.constants.DriveConstants.FrontRightModule;

import java.util.Map;

import org.littletonrobotics.urcl.URCL;

import com.revrobotics.util.StatusLogger;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    //URCL already logs the sparks, don't double up with REV's .revlog files. has to run before any spark is created
    StatusLogger.disableAutoLogging();

    //sysid reads its data out of these .wpilog files
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    //logs every spark's CAN status frames into the same .wpilog, for drive/turn sysid
    URCL.start(Map.of(
      FrontLeftModule.kDriveCANID, "FL-Drive", FrontLeftModule.kTurnCANID, "FL-Turn",
      FrontRightModule.kDriveCANID, "FR-Drive", FrontRightModule.kTurnCANID, "FR-Turn",
      BackLeftModule.kDriveCANID, "BL-Drive", BackLeftModule.kTurnCANID, "BL-Turn",
      BackRightModule.kDriveCANID, "BR-Drive", BackRightModule.kTurnCANID, "BR-Turn"),
      DataLogManager.getLog());

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
