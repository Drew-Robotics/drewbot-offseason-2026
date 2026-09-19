package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

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
            Units.MetersPerSecond.of(m_xVel.getAsDouble()*DriveConstants.maxSpeed.in(Units.MetersPerSecond)), 
            Units.MetersPerSecond.of(m_yVel.getAsDouble()*DriveConstants.maxSpeed.in(Units.MetersPerSecond)), 
            Units.RadiansPerSecond.of(m_rotVel.getAsDouble()*2*Math.PI));
    }

    public void end() {}

    public boolean isFinished() {
        return false;
    }
}