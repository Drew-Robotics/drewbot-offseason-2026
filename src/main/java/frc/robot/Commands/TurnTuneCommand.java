package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.TurnMotor;

//live tunes all 4 turn motors at once with shared gains. everything lives under TurnTuning/ in NT:
//  inputs:  TargetDeg, kP, kI, kD
//  outputs: <FL|FR|BL|BR>/CurrentDeg, SetpointDeg, ErrorDeg (graph these), plus output and gains read back off each flex
public class TurnTuneCommand extends Command {
    private static final String kTable = "TurnTuning/";
    private static final String[] kModuleNames = {"FL", "FR", "BL", "BR"};

    private double m_lastP, m_lastI, m_lastD;

    public TurnTuneCommand() {
        addRequirements(DriveSubsystem.getInstance());

        SmartDashboard.putNumber(kTable + "TargetDeg", 0);
    }

    @Override
    public void initialize() {
        //seed NT from what FL's flex is really running; setting gains below pushes them to all 4
        TurnMotor seed = DriveSubsystem.getInstance().getModule(0).getTurnMotor();
        m_lastP = seed.getP();
        m_lastI = seed.getI();
        m_lastD = seed.getD();
        SmartDashboard.putNumber(kTable + "kP", m_lastP);
        SmartDashboard.putNumber(kTable + "kI", m_lastI);
        SmartDashboard.putNumber(kTable + "kD", m_lastD);
        publishFlexGains();
    }

    @Override
    public void execute() {
        DriveSubsystem drive = DriveSubsystem.getInstance();

        double p = SmartDashboard.getNumber(kTable + "kP", m_lastP);
        double i = SmartDashboard.getNumber(kTable + "kI", m_lastI);
        double d = SmartDashboard.getNumber(kTable + "kD", m_lastD);
        //configure() blocks on CAN, so only push when something actually changed
        if (p != m_lastP || i != m_lastI || d != m_lastD) {
            for (int m = 0; m < 4; m++) {
                drive.getModule(m).getTurnMotor().setPID(p, i, d);
            }
            m_lastP = p;
            m_lastI = i;
            m_lastD = d;
            publishFlexGains();
        }

        Rotation2d target = Rotation2d.fromDegrees(SmartDashboard.getNumber(kTable + "TargetDeg", 0));
        for (int m = 0; m < 4; m++) {
            drive.getModule(m).setTurnAngleDirect(target);

            TurnMotor turnMotor = drive.getModule(m).getTurnMotor();
            String prefix = kTable + kModuleNames[m] + "/";
            Rotation2d current = turnMotor.getAngle();
            Rotation2d setpoint = turnMotor.getTargetAngle();
            SmartDashboard.putNumber(prefix + "CurrentDeg", current.getDegrees());
            SmartDashboard.putNumber(prefix + "SetpointDeg", setpoint.getDegrees());
            SmartDashboard.putNumber(prefix + "ErrorDeg", setpoint.minus(current).getDegrees());
            SmartDashboard.putNumber(prefix + "AppliedOutput", turnMotor.getAppliedOutput());
            SmartDashboard.putNumber(prefix + "CurrentAmps", turnMotor.getOutputCurrent());
        }
    }

    //gain reads are blocking CAN calls, so only confirm them after a change instead of every loop
    private void publishFlexGains() {
        for (int m = 0; m < 4; m++) {
            TurnMotor turnMotor = DriveSubsystem.getInstance().getModule(m).getTurnMotor();
            String prefix = kTable + kModuleNames[m] + "/";
            SmartDashboard.putNumber(prefix + "FlexP", turnMotor.getP());
            SmartDashboard.putNumber(prefix + "FlexI", turnMotor.getI());
            SmartDashboard.putNumber(prefix + "FlexD", turnMotor.getD());
        }
    }

    @Override
    public void end(boolean interrupted) {
        for (int m = 0; m < 4; m++) {
            DriveSubsystem.getInstance().getModule(m).stopDrive();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
