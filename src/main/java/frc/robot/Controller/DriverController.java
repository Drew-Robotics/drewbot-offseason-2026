package frc.robot.controller;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriverController extends Controller{
    public DriverController (int port) {
        super(port);
    }

    //robot +x is forward, +y is left, rotation is ccw-positive; the xbox sticks report
    //y negative when pushed forward and x positive when pushed right, so both get negated
    public double getDriveX() {
        return applyDeadband(-getLeftY());
    }

    public double getDriveY() {
        return applyDeadband(-getLeftX());
    }

    public double getDriveRotVel() {
        return applyDeadband(-getRightX());
    }

    public double getDriveRot() {
        return Math.atan2(-getRightX(), -getRightY());
    }

    public boolean getYButton() {
        return y().getAsBoolean();
    }

    public boolean getXButton() {
        return x().getAsBoolean();
    }

    public boolean getBButton() {
        return b().getAsBoolean();
    }
}
