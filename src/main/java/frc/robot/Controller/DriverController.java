package frc.robot.Controller;

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
        return applyDeadband(getRightX());
    }
}
