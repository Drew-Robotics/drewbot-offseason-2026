package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Controller extends CommandXboxController{
    private static final double m_deadband = 0.1; //This should 100% be in a constants file but I'm being lazy af rn

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
