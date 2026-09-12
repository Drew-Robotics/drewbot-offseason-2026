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
    
    public SwerveModule (TurnMotor turnMotor, DriveMotor driveMotor) {
        
        m_turnMotor = turnMotor;
        m_driveMotor = driveMotor;
    }

    private Rotation2d angleRelativeToRobot(Rotation2d angle){
        return angle.minus(m_offset);
    }

    private Rotation2d angleRelativeToModule(Rotation2d angle){ //just like resets an angle that's already relative to the robot
        return angle.plus(m_offset);
    }

    //PUBLIC METHODS:
    //bro who tf is enough of a tryhard to specify where the public methods start we're doing too much

    public void setState(SwerveModuleState moduleState){
        SwerveModuleState targetModuleState = new SwerveModuleState(
            moduleState.speedMetersPerSecond, 
            angleRelativeToModule(moduleState.angle)
            );
        
        targetModuleState.optimize(m_turnMotor.getAngle());

        m_turnMotor.setAngle(targetModuleState.angle);
        m_driveMotor.setVelocity(Units.MetersPerSecond.of(targetModuleState.speedMetersPerSecond));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveSpeed(), angleRelativeToRobot(m_turnMotor.getAngle()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDistance(), angleRelativeToRobot(m_turnMotor.getAngle()));
    }

    public LinearVelocity getDriveSpeed() {
        return m_driveMotor.getVelocity();
    }

    public Distance getDistance() {
        return m_driveMotor.getPosition();
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
