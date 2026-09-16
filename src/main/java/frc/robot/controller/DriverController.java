package frc.robot.controller;

public class DriverController extends Controller{
    public DriverController (int port) {
        super(port);
    }

    //WPILib field axes: +x is forward, +y is left, +rot is CCW. xbox sticks are the opposite on all three

    public double getDriveX() {
        return -applyDeadband(getLeftY());
    }

    public double getDriveY() {
        return -applyDeadband(getLeftX());
    }

    public double getDriveRot() {
        return -applyDeadband(getRightX());//I wanna make it so that this has multiple aiming options
    }
}
