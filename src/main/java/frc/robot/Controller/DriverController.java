package frc.robot.controller;

public class DriverController extends Controller{
    public DriverController (int port) {
        super(port);
    }

    public double getDriveX() {
        return applyDeadband(getLeftX());
    }

    public double getDriveY() {
        return applyDeadband(getLeftY());
    }

    public double getDriveRot() {
        return applyDeadband(getRightX());//I wanna make it so that this has multiple aiming options
    }
}
