package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class SwerveModule {
    private String m_name;

    private final TurnMotor m_turnMotor;
    private final DriveMotor m_driveMotor;

    Rotation2d m_offset; //we use rotation2d bc that's what the swervemodulestate methods intake
    
    private SwerveModuleState m_lastCommanded = new SwerveModuleState();

    public SwerveModule (String name, TurnMotor turnMotor, DriveMotor driveMotor, Rotation2d offset) {
        m_name = name;

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
        m_driveMotor.setOpenLoop(Units.MetersPerSecond.of(targetModuleState.speedMetersPerSecond));
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

    public Rotation2d getAngle(){
        return m_turnMotor.getAngle();
    }

    //skips optimize() so the turn motor goes exactly where it's told, for tuning
    public void setTurnAngleDirect(Rotation2d angle){
        m_turnMotor.setAngle(angle);
        m_driveMotor.setOpenLoop(Units.MetersPerSecond.of(0));
    }

    public void stopDrive(){
        m_driveMotor.setOpenLoop(Units.MetersPerSecond.of(0));
    }

    public TurnMotor getTurnMotor(){
        return m_turnMotor;
    }

    public void encoderVoltageCheck(){
        m_turnMotor.encoderVoltageCheck();
    }

    public String getName() {
        return m_name;
    }

    public void periodic() {
        m_turnMotor.periodic();
    }
}
