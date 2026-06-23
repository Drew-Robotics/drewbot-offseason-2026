package frc.robot.Subsystems;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase{
    private final List<SwerveModule> m_swerveModules;
    private final Pigeon2 m_gyro;
    private final SwerveDrivePoseEstimator m_poser;

    private static DriveSubsystem m_instance;

    public DriveSubsystem () {
        m_swerveModules = List.of(
            new SwerveModule(
                "FrontLeft", 
                new TurnMotor(
                    DriveConstants.FrontLeftModule.kTurnCANID, 
                    DriveConstants.FrontLeftModule.kTurnInverted
                ),
                new DriveMotor(
                    DriveConstants.FrontLeftModule.kDriveCANID, 
                    DriveConstants.FrontLeftModule.kDriveInverted
                ),
                DriveConstants.FrontLeftModule.kOffset
            ),
            new SwerveModule(
                "FrontRight", 
                new TurnMotor(
                    DriveConstants.FrontRightModule.kTurnCANID, 
                    DriveConstants.FrontRightModule.kTurnInverted
                ),
                new DriveMotor(
                    DriveConstants.FrontRightModule.kDriveCANID, 
                    DriveConstants.FrontRightModule.kDriveInverted
                ),
                DriveConstants.FrontRightModule.kOffset
            ),
            new SwerveModule(
                "BackLeft", 
                new TurnMotor(
                    DriveConstants.BackLeftModule.kTurnCANID, 
                    DriveConstants.BackLeftModule.kTurnInverted
                ),
                new DriveMotor(
                    DriveConstants.BackLeftModule.kDriveCANID, 
                    DriveConstants.BackLeftModule.kDriveInverted
                ),
                DriveConstants.BackLeftModule.kOffset
            ),
            new SwerveModule(
                "BackRight", 
                new TurnMotor(
                    DriveConstants.BackRightModule.kTurnCANID, 
                    DriveConstants.BackRightModule.kTurnInverted
                ),
                new DriveMotor(
                    DriveConstants.BackRightModule.kDriveCANID, 
                    DriveConstants.BackRightModule.kDriveInverted
                ),
                DriveConstants.BackRightModule.kOffset
            )
        );

        m_gyro = new Pigeon2(DriveConstants.Gyroscope.kCanID);

        m_poser = new SwerveDrivePoseEstimator(
            DriveConstants.kKinematics, 
            Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()), 
            getModulePositions(), 
            new Pose2d());  
    }

    public static final DriveSubsystem getInstance() {
        if(m_instance==null){
            m_instance = new DriveSubsystem();
        }
        return m_instance;
    }

    private void setSwerveModuleStates(List<SwerveModuleState> states){
        for(int i = 0; i<4; i++){
            m_swerveModules.get(i).setState(states.get(i));
        }//this ts only works because both the kinematics list and m_swervemodules list go in the same order (fl fr bl br) and I use that order everywhere I call this, it's cooked and you should code a better function if you're doing this yourslef
    }

    public void setChassisSpeed(ChassisSpeeds speeds){
        SwerveModuleState[] states = DriveConstants.kKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.maxSpeed);
        List<SwerveModuleState> statesList = Arrays.asList(states);
        setSwerveModuleStates(statesList);
    }

    private SwerveModulePosition[] getModulePositions(){
        return m_swerveModules.stream()
            .map(SwerveModule -> SwerveModule.getPosition())
            .toArray(SwerveModulePosition[]::new);
    }

    public void fieldOrientedDrive (LinearVelocity xVel, LinearVelocity yVel, AngularVelocity rotVel) {
        ChassisSpeeds speeds = new ChassisSpeeds(xVel.in(Units.MetersPerSecond), yVel.in(Units.MetersPerSecond), rotVel.in(Units.RadiansPerSecond));
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, m_gyro.getRotation2d());
        setChassisSpeed(speeds);
    }

    @Override
    public void periodic(){
        super.periodic();

        m_poser.update(new Rotation2d(m_gyro.getYaw().getValueAsDouble()), getModulePositions());
    }
}
