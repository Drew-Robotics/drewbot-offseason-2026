package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.MiscConstants;

public class Controller extends CommandXboxController{
    private static final double m_deadband = MiscConstants.kDeadBand;

    public Controller (int port){
        super(port);
    }

    public static double applyDeadband (double val) {
        return Math.abs(val) <= m_deadband ? 0
        : Math.signum(val) * 
            (Math.abs(val) - m_deadband)
            / (1 - m_deadband);
    }
}
