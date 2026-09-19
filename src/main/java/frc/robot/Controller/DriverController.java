package frc.robot.controller;

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

    public double getDriveRot() {
        return applyDeadband(-getRightX());
    }
}
