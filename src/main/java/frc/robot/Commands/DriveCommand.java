package frc.robot.Commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;

public class DriveCommand extends Command {
    private DoubleSupplier m_xVel;
    private DoubleSupplier m_yVel;
    private DoubleSupplier m_rotVel;

    public DriveCommand(DoubleSupplier xVel, DoubleSupplier yVel, DoubleSupplier rotVel){
        m_xVel = xVel;
        m_yVel = yVel;
        m_rotVel = rotVel;

        addRequirements(DriveSubsystem.getInstance());
    }

    public void initialize() {}

    public void execute() {
        DriveSubsystem.getInstance().fieldOrientedDrive(
            Units.MetersPerSecond.of(m_xVel.getAsDouble()), 
            Units.MetersPerSecond.of(m_yVel.getAsDouble()), 
            Units.RadiansPerSecond.of(m_rotVel.getAsDouble()));
    }

    public void end() {}

    public boolean isFinished() {
        return false;
    }
}