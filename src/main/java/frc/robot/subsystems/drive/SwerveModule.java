package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class SwerveModule {
    String m_name;

    TurnMotor m_turnMotor;
    DriveMotor m_driveMotor;

    Rotation2d m_offset; //we use rotation2d bc that's what the swervemodulestate methods intake
    
    private SwerveModuleState m_lastCommanded = new SwerveModuleState();

    public SwerveModule (TurnMotor turnMotor, DriveMotor driveMotor, Rotation2d offset) {
        m_turnMotor = turnMotor;
        m_driveMotor = driveMotor;

        m_offset = offset;
    }

    //PUBLIC METHODS:
    //bro who tf is enough of a tryhard to specify where the public methods start we're doing too much

    public void setState(SwerveModuleState moduleState){
        SwerveModuleState targetModuleState = new SwerveModuleState(
            moduleState.speedMetersPerSecond, 
            moduleState.angle
            );
        
        targetModuleState.optimize(m_turnMotor.getAngle());

        m_lastCommanded = targetModuleState;
        m_turnMotor.setAngle(targetModuleState.angle);
        m_driveMotor.setVelocity(Units.MetersPerSecond.of(targetModuleState.speedMetersPerSecond));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveSpeed(), m_turnMotor.getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDistance(), m_turnMotor.getAngle());
    }

    public LinearVelocity getDriveSpeed() {
        return m_driveMotor.getVelocity();
    }

    public Distance getDistance() {
        return m_driveMotor.getPosition();
    }

    public SwerveModuleState getLastCommanded(){
        return m_lastCommanded;
    }

    public Rotation2d getAbsoluteAngle(){
        return m_turnMotor.getAngle();
    }

    public void encoderVoltageCheck(){
        m_turnMotor.encoderVoltageCheck();
    }

    public void periodic() {
        m_turnMotor.periodic();
    }
}
