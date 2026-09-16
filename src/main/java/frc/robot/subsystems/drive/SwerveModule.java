package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.DriveConstants.kBodyMeasures;

public class SwerveModule {
    private final String m_name;

    private final TurnMotor m_turnMotor;
    private final DriveMotor m_driveMotor;

    private static final double kResyncPeriodSecs = 1.0;
    private static final double kResyncMaxVelocityRadPerSec = 0.1; //don't reseed while the module is still spinning
    private final Timer m_resyncTimer = new Timer();

    public SwerveModule(String name, TurnMotor turnMotor, DriveMotor driveMotor) {
        m_name = name;
        m_turnMotor = turnMotor;
        m_driveMotor = driveMotor;
        m_resyncTimer.start();
    }

    /** All angles in and out of here are relative to the robot. */
    public void setState(SwerveModuleState moduleState) {
        SwerveModuleState targetModuleState = new SwerveModuleState(moduleState.speedMetersPerSecond, moduleState.angle);
        Rotation2d currentAngle = getAngle();

        targetModuleState.optimize(currentAngle);
        targetModuleState.cosineScale(currentAngle); //slow the wheel down while it's still pointed the wrong way

        m_turnMotor.setAngle(targetModuleState.angle);
        m_driveMotor.setVelocity(Units.MetersPerSecond.of(targetModuleState.speedMetersPerSecond));
    }

    /** Open loop drive at a fixed module angle, for sysid + characterization. */
    public void runDriveCharacterization(Voltage driveVoltage, Rotation2d angle) {
        m_turnMotor.setAngle(angle);
        m_driveMotor.setVoltage(driveVoltage);
    }

    /** Open loop turn with the drive motor stopped, for sysid. */
    public void runTurnCharacterization(Voltage turnVoltage) {
        m_driveMotor.setVoltage(Units.Volts.of(0));
        m_turnMotor.setVoltage(turnVoltage);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveSpeed(), getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDistance(), getAngle());
    }

    public Rotation2d getAngle() {
        return m_turnMotor.getAngle();
    }

    public LinearVelocity getDriveSpeed() {
        return m_driveMotor.getVelocity();
    }

    public Distance getDistance() {
        return m_driveMotor.getPosition();
    }

    /** How far the wheel has spun in radians, using the radius in constants. */
    public double getWheelRadians() {
        return getDistance().in(Units.Meters) / kBodyMeasures.kWheelRadius.in(Units.Meters);
    }

    public DriveMotor getDriveMotor() {
        return m_driveMotor;
    }

    public TurnMotor getTurnMotor() {
        return m_turnMotor;
    }

    public String getName() {
        return m_name;
    }

    public void periodic() {
        if (m_turnMotor.checkAndClearReset()) {
            DriverStation.reportWarning(m_name + " turn motor reset, reseeding from absolute encoder", false);
            m_turnMotor.seedFromAbsolute();
        }
        //the analog value isn't there yet the instant the flex boots, so keep reseeding while disabled
        if (DriverStation.isDisabled()
            && m_resyncTimer.advanceIfElapsed(kResyncPeriodSecs)
            && Math.abs(m_turnMotor.getVelocity().in(Units.RadiansPerSecond)) < kResyncMaxVelocityRadPerSec) {
            m_turnMotor.seedFromAbsolute();
        }

        SmartDashboard.putNumber("Drive/" + m_name + "/Encoder Volts", m_turnMotor.getEncoderVoltage());
        SmartDashboard.putNumber("Drive/" + m_name + "/Raw Absolute Angle Deg", m_turnMotor.getRawAbsoluteAngle().getDegrees());
        SmartDashboard.putNumber("Drive/" + m_name + "/Absolute Angle Deg", m_turnMotor.getAbsoluteAngle().getDegrees());
        SmartDashboard.putNumber("Drive/" + m_name + "/Angle Deg", getAngle().getDegrees());
        SmartDashboard.putNumber("Drive/" + m_name + "/Speed MPS", getDriveSpeed().in(Units.MetersPerSecond));
    }
}
