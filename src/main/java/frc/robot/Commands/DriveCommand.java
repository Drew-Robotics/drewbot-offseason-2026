package frc.robot.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class DriveCommand extends Command {
    private DoubleSupplier m_xVel;
    private DoubleSupplier m_yVel;
    private DoubleSupplier m_rotVel;
    private DoubleSupplier m_rot;

    private BooleanSupplier m_setFieldRelativeDrive;
    private BooleanSupplier m_setStandardDrive;
    private Boolean m_useFieldRelativeDrive = false;

    private BooleanSupplier m_zeroRobot;

    public DriveCommand(DoubleSupplier xVel, DoubleSupplier yVel, DoubleSupplier rotVel, DoubleSupplier rot, BooleanSupplier yButton, BooleanSupplier xButton, BooleanSupplier bButton){
        m_xVel = xVel;
        m_yVel = yVel;
        m_rotVel = rotVel;
        m_rot = rot;
        
        m_zeroRobot = yButton;

        m_setFieldRelativeDrive = bButton;
        m_setStandardDrive = xButton;

        addRequirements(DriveSubsystem.getInstance());
    }

    public void initialize() {}

    public void execute() {
        if(m_setFieldRelativeDrive.getAsBoolean()) {
            m_useFieldRelativeDrive = true;
        }
        else if (m_setStandardDrive.getAsBoolean()) {
            m_useFieldRelativeDrive = false;
        }

        if(m_useFieldRelativeDrive){
            DriveSubsystem.getInstance().fieldOrientedDrive(
                DriveConstants.kMaxVel.times(m_xVel.getAsDouble()), 
                DriveConstants.kMaxVel.times(m_yVel.getAsDouble()),
                Optional.empty(),
                Optional.of(m_rot.getAsDouble()));
        }
        else {
            DriveSubsystem.getInstance().fieldOrientedDrive(
                DriveConstants.kMaxVel.times(m_xVel.getAsDouble()), 
                DriveConstants.kMaxVel.times(m_yVel.getAsDouble()),
                Optional.of(DriveConstants.kMaxRotVel.times(m_rotVel.getAsDouble())),
                Optional.empty());
        }

        if(m_zeroRobot.getAsBoolean()) {
            DriveSubsystem.getInstance().setZero();
        }
    }

    public void end() {}

    public boolean isFinished() {
        return false;
    }
}