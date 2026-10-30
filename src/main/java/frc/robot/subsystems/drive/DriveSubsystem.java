package frc.robot.subsystems.drive;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.vision.Camera;
import frc.robot.subsystems.vision.VisionSubsystem;

public class DriveSubsystem extends SubsystemBase{
    private final List<SwerveModule> m_swerveModules;
    private final Pigeon2 m_gyro;
    private final SwerveDrivePoseEstimator m_poser;

    private final PIDController m_turnPid;

    private static DriveSubsystem m_instance;

    private NetworkTable m_table;
    private StructPublisher<Pose2d> m_posePub;
    private Field2d m_dashField;

    public DriveSubsystem () {
        m_dashField = new Field2d();
        m_table = NetworkTableInstance.getDefault().getTable("busterPosey");
        m_posePub = m_table.getStructTopic("Robot Pose", Pose2d.struct).publish();
        
        m_swerveModules = List.of(
            new SwerveModule(
                "Front Left",
                new TurnMotor(
                    "Front Left",
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
                "Front Right",
                new TurnMotor(
                    "Front Right",
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
                "Back Left",
                new TurnMotor(
                    "Back Left",
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
                "Back Right",
                new TurnMotor(
                    "Back Right",
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
        
        m_turnPid = new PIDController(1, 0, 0);
        m_turnPid.enableContinuousInput(-Math.PI, Math.PI);

        publishInit();
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

    public void setZero(){
        m_gyro.reset();
        //m_poser.resetRotation(Rotation2d.fromRadians(0));
    }

    public void setChassisSpeed(ChassisSpeeds speeds){
        SwerveModuleState[] states = DriveConstants.kKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxVel);
        List<SwerveModuleState> statesList = Arrays.asList(states);
        setSwerveModuleStates(statesList);
    }

    public void fieldOrientedDrive(LinearVelocity xVel, LinearVelocity yVel, Optional<AngularVelocity> rotVel, Optional<Double> rot) {
        AngularVelocity rv;
        if(!rotVel.isEmpty()) {
            rv = rotVel.get();
        }
        else if(!rot.isEmpty()) {
            rv = Units.RadiansPerSecond.of(m_turnPid.calculate(m_gyro.getRotation2d().getRadians(), rot.get()));
        }
        else {
            rv = Units.RadiansPerSecond.of(0);
        }
        //rv = rotVel;

        ChassisSpeeds speeds = new ChassisSpeeds(xVel.in(Units.MetersPerSecond), yVel.in(Units.MetersPerSecond), rv.in(Units.RadiansPerSecond));
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()));
        setChassisSpeed(speeds);
        SmartDashboard.putNumber("rotVel", rv.in(Units.RadiansPerSecond));
    }

    //index order is fl fr bl br, same as everywhere else
    public SwerveModule getModule(int index){
        return m_swerveModules.get(index);
    }

    public Pose2d getPose () {
        return m_poser.getEstimatedPosition();
    }

    private void poseVisionIncorporation() {
        List<Camera> cams = VisionSubsystem.getInstance().getCameras();

        for(Camera cam : cams) {
            if(!cam.getPoseEstimation().isEmpty()){
                m_poser.addVisionMeasurement(
                    cam.getPoseEstimation().get().estimatedPose.toPose2d(), 
                    cam.getPoseEstimation().get().timestampSeconds,
                    cam.getStandardDeviation(getPose()).get());
    }   }   }


    protected void publishInit() {
        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable (SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                for(SwerveModule swerveModule : m_swerveModules){
                    builder.addDoubleProperty(
                        swerveModule.getName()+" Angle", 
                        () -> swerveModule.getAngle().getRadians(), 
                        null);
                    builder.addDoubleProperty(
                        swerveModule.getName()+" Velocity", 
                        () -> swerveModule.getDriveSpeed().in(Units.MetersPerSecond), 
                        null);
                }

                builder.addDoubleProperty("Robot Angle", () -> m_gyro.getYaw().getValueAsDouble()*2*Math.PI/360, null);
            }   });

        
    }


    @Override
    public void periodic(){
        super.periodic();
        for(int i = 0; i < 4; i++) {
            m_swerveModules.get(i).periodic();
        }

        String[] names = {"Front Left","Front Right","Back Left","Back Right"};
        SmartDashboard.putNumber("GyroYawDeg", m_gyro.getYaw().getValueAsDouble());
        for(int i = 0; i < 4; i++){
            SwerveModule mod = m_swerveModules.get(i);
            SmartDashboard.putNumber(names[i] + "CmdSpeed", mod.getLastCommanded().speedMetersPerSecond);
            SmartDashboard.putNumber(names[i] + "CmdAngle", mod.getLastCommanded().angle.getRadians());
            SmartDashboard.putNumber(names[i] + "ActualVel", mod.getDriveSpeed().in(Units.MetersPerSecond));
            SmartDashboard.putNumber(names[i] + "ENC", m_swerveModules.get(1).getTurnMotor().getAngle().getDegrees());
        }

        m_poser.update(Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()), getModulePositions());
        //poseVisionIncorporation();

        m_posePub.accept(getPose());
        m_dashField.setRobotPose(getPose());
        SmartDashboard.putData("robot field", m_dashField);
    }
}
