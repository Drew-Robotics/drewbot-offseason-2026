package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

/** Field relative teleop drive. Suppliers are -1 to 1, already in WPILib axes (+x forward, +y left, +rot CCW). */
public class DriveCommand extends Command {
    private final DriveSubsystem m_drive;
    private final DoubleSupplier m_xVel;
    private final DoubleSupplier m_yVel;
    private final DoubleSupplier m_rotVel;

    public DriveCommand(DriveSubsystem drive, DoubleSupplier xVel, DoubleSupplier yVel, DoubleSupplier rotVel) {
        m_drive = drive;
        m_xVel = xVel;
        m_yVel = yVel;
        m_rotVel = rotVel;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        m_drive.driveFieldRelative(new ChassisSpeeds(
            m_xVel.getAsDouble() * DriveConstants.kMaxSpeed.in(Units.MetersPerSecond),
            m_yVel.getAsDouble() * DriveConstants.kMaxSpeed.in(Units.MetersPerSecond),
            m_rotVel.getAsDouble() * DriveConstants.kMaxAngularSpeed.in(Units.RadiansPerSecond)));
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }
}
