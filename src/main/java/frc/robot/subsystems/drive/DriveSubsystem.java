package frc.robot.subsystems.drive;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.fasterxml.jackson.databind.util.RootNameLookup;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.vision.Camera;
import frc.robot.subsystems.vision.VisionSubsystem;

public class DriveSubsystem extends SubsystemBase{
    private final List<SwerveModule> m_swerveModules;
    private final Pigeon2 m_gyro;
    private final SwerveDrivePoseEstimator m_poser;

    private static DriveSubsystem m_instance;

    public DriveSubsystem () {
        m_swerveModules = List.of(
            new SwerveModule(
                new TurnMotor(
                    "FL",
                    DriveConstants.FrontLeftModule.kTurnCANID, 
                    DriveConstants.FrontLeftModule.kTurnInverted,
                    DriveConstants.FrontLeftModule.kOffset
                ),
                new DriveMotor(
                    DriveConstants.FrontLeftModule.kDriveCANID, 
                    DriveConstants.FrontLeftModule.kDriveInverted
                ),
                DriveConstants.FrontLeftModule.kOffset
            ),
            new SwerveModule(
                new TurnMotor(
                    "FR",
                    DriveConstants.FrontRightModule.kTurnCANID, 
                    DriveConstants.FrontRightModule.kTurnInverted,
                    DriveConstants.FrontRightModule.kOffset
                ),
                new DriveMotor(
                    DriveConstants.FrontRightModule.kDriveCANID, 
                    DriveConstants.FrontRightModule.kDriveInverted
                ),
                DriveConstants.FrontRightModule.kOffset
            ),
            new SwerveModule(
                new TurnMotor(
                    "BL",
                    DriveConstants.BackLeftModule.kTurnCANID, 
                    DriveConstants.BackLeftModule.kTurnInverted,
                    DriveConstants.BackLeftModule.kOffset
                ),
                new DriveMotor(
                    DriveConstants.BackLeftModule.kDriveCANID, 
                    DriveConstants.BackLeftModule.kDriveInverted
                ),
                DriveConstants.BackLeftModule.kOffset
            ),
            new SwerveModule(
                new TurnMotor(
                    "BR",
                    DriveConstants.BackRightModule.kTurnCANID, 
                    DriveConstants.BackRightModule.kTurnInverted,
                    DriveConstants.BackRightModule.kOffset
                ),
                new DriveMotor(
                    DriveConstants.BackRightModule.kDriveCANID, 
                    DriveConstants.BackRightModule.kDriveInverted
                ),
                DriveConstants.BackRightModule.kOffset
            )
        );

        m_gyro = new Pigeon2(DriveConstants.GyroscopeConstants.kCanID);
        m_gyro.setYaw(0);

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

    private SwerveModulePosition[] getModulePositions(){
    return m_swerveModules.stream()
        .map(SwerveModule -> SwerveModule.getPosition())
        .toArray(SwerveModulePosition[]::new);
    }

    public void setChassisSpeed(ChassisSpeeds speeds){
        SwerveModuleState[] states = DriveConstants.kKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.maxSpeed);
        List<SwerveModuleState> statesList = Arrays.asList(states);
        setSwerveModuleStates(statesList);
    }

    public void fieldOrientedDrive(LinearVelocity xVel, LinearVelocity yVel, AngularVelocity rotVel) {
        ChassisSpeeds speeds = new ChassisSpeeds(xVel.in(Units.MetersPerSecond), yVel.in(Units.MetersPerSecond), rotVel.in(Units.RadiansPerSecond));
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()));
        setChassisSpeed(speeds);
    }

    //index order is fl fr bl br, same as everywhere else
    public SwerveModule getModule(int index){
        return m_swerveModules.get(index);
    }

    public Pose2d getPose () {
        return m_poser.getEstimatedPosition();
    }


    @Override
    public void periodic(){
        super.periodic();
        for(int i = 0; i < 4; i++) {
            m_swerveModules.get(i).periodic();
        }
        m_swerveModules.get(0).encoderVoltageCheck();
        SmartDashboard.putNumber("FL ENC", m_swerveModules.get(0).m_turnMotor.getAngle().getDegrees());
        SmartDashboard.putNumber("FR ENC", m_swerveModules.get(1).m_turnMotor.getAngle().getDegrees());
        SmartDashboard.putNumber("BL ENC", m_swerveModules.get(2).m_turnMotor.getAngle().getDegrees());
        SmartDashboard.putNumber("BR ENC", m_swerveModules.get(3).m_turnMotor.getAngle().getDegrees());

        String[] names = {"FL","FR","BL","BR"};
        SmartDashboard.putNumber("GyroYawDeg", m_gyro.getYaw().getValueAsDouble());
        for(int i = 0; i < 4; i++){
            SwerveModule mod = m_swerveModules.get(i);
            SmartDashboard.putNumber(names[i] + "CmdSpeed", mod.getLastCommanded().speedMetersPerSecond);
            SmartDashboard.putNumber(names[i] + "CmdAngle", mod.getLastCommanded().angle.getRadians());
            SmartDashboard.putNumber(names[i] + "ActualVel", mod.getDriveSpeed().in(Units.MetersPerSecond));
        }

        m_poser.update(Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()), getModulePositions());
    }

    private void poseVisionIncorporation() {
        List<Camera> cams = VisionSubsystem.getInstance().getCameras();

        for(Camera cam : cams) {
            m_poser.addVisionMeasurement(
                cam.getPoseEstimation().get().estimatedPose.toPose2d(), 
                cam.getPoseEstimation().get().timestampSeconds,
                cam.getStandardDeviation(getPose()).get());
        }
    }
}
