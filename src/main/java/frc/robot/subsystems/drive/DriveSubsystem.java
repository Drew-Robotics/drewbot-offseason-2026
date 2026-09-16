package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.DriveConstants.BackLeftModule;
import frc.robot.constants.DriveConstants.BackRightModule;
import frc.robot.constants.DriveConstants.CharacterizationConstants;
import frc.robot.constants.DriveConstants.DriveMotorConstants;
import frc.robot.constants.DriveConstants.FrontLeftModule;
import frc.robot.constants.DriveConstants.FrontRightModule;
import frc.robot.constants.DriveConstants.HeadingConstants;
import frc.robot.constants.DriveConstants.kBodyMeasures;

public class DriveSubsystem extends SubsystemBase {
    private static final double kLoopPeriodSecs = 0.02;

    //order is FL, FR, BL, BR everywhere, same as kKinematics
    private final List<SwerveModule> m_swerveModules;
    private final Pigeon2 m_gyro;
    private final SwerveDrivePoseEstimator m_poser;
    private final PIDController m_headingController;

    private final SysIdRoutine m_driveSysId;
    private final SysIdRoutine m_turnSysId;
    private final SysIdRoutine m_rotationSysId;
    private double m_lastRotationRateRequest = 0; //rad/s, "voltage" for the rotation sysid

    //until someone explicitly sets the pose (zero heading, auto start pose), face the robot away from our
    //alliance wall. otherwise on red the robot boots thinking it faces 0deg and field relative is backwards
    private boolean m_poseSetExplicitly = false;
    private Optional<Alliance> m_lastAlliance = Optional.empty();

    private final Field2d m_field = new Field2d();
    private final StructArrayPublisher<SwerveModuleState> m_measuredStatesPub = NetworkTableInstance.getDefault()
        .getStructArrayTopic("Drive/MeasuredStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> m_setpointStatesPub = NetworkTableInstance.getDefault()
        .getStructArrayTopic("Drive/SetpointStates", SwerveModuleState.struct).publish();
    private final StructPublisher<Pose2d> m_posePub = NetworkTableInstance.getDefault()
        .getStructTopic("Drive/Pose", Pose2d.struct).publish();

    public DriveSubsystem() {
        m_swerveModules = List.of(
            new SwerveModule("FL",
                new TurnMotor(FrontLeftModule.kTurnCANID, FrontLeftModule.kTurnInverted, FrontLeftModule.kOffset),
                new DriveMotor(FrontLeftModule.kDriveCANID, FrontLeftModule.kDriveInverted)),
            new SwerveModule("FR",
                new TurnMotor(FrontRightModule.kTurnCANID, FrontRightModule.kTurnInverted, FrontRightModule.kOffset),
                new DriveMotor(FrontRightModule.kDriveCANID, FrontRightModule.kDriveInverted)),
            new SwerveModule("BL",
                new TurnMotor(BackLeftModule.kTurnCANID, BackLeftModule.kTurnInverted, BackLeftModule.kOffset),
                new DriveMotor(BackLeftModule.kDriveCANID, BackLeftModule.kDriveInverted)),
            new SwerveModule("BR",
                new TurnMotor(BackRightModule.kTurnCANID, BackRightModule.kTurnInverted, BackRightModule.kOffset),
                new DriveMotor(BackRightModule.kDriveCANID, BackRightModule.kDriveInverted))
        );

        if (kBodyMeasures.kDriveBaseRadius.in(Meters) == 0) {
            DriverStation.reportError("DriveConstants.kBodyMeasures wheelbase/track width are 0, set them before driving!", false);
        }

        m_gyro = new Pigeon2(DriveConstants.GyroscopeConstants.kCanID);

        m_poser = new SwerveDrivePoseEstimator(
            DriveConstants.kKinematics,
            m_gyro.getRotation2d(),
            getModulePositions(),
            Pose2d.kZero);

        m_headingController = new PIDController(
            HeadingConstants.PID.kP, HeadingConstants.PID.kI, HeadingConstants.PID.kD);
        m_headingController.enableContinuousInput(-Math.PI, Math.PI);

        //drive + turn motor data comes from URCL (Spark CAN frames at 10ms), so the log consumer is null
        //and the routine only records the test state
        m_driveSysId = new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(4), null), //default 7V dynamic step runs away fast, disable early if you run out of room
            new SysIdRoutine.Mechanism(
                voltage -> m_swerveModules.forEach(module -> module.runDriveCharacterization(voltage, Rotation2d.kZero)),
                null,
                this,
                "drive"));

        m_turnSysId = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.per(Second).of(0.5), Volts.of(3), Seconds.of(10)),
            new SysIdRoutine.Mechanism(
                voltage -> m_swerveModules.get(CharacterizationConstants.kTurnSysIdModuleIndex).runTurnCharacterization(voltage),
                null,
                this,
                "turn"));

        //CTRE's trick: the "voltage" sysid sends is really a rotation rate in rad/s. analyze as a position
        //loop and the kP/kD sysid spits out go straight into the heading PIDController. this one logs from the
        //rio on purpose, the heading PID runs on the rio at 20ms so that's the data it should be tuned on
        m_rotationSysId = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.per(Second).of(Math.PI / 6), Volts.of(Math.PI), Seconds.of(10)),
            new SysIdRoutine.Mechanism(
                rate -> {
                    m_lastRotationRateRequest = rate.in(Volts);
                    runVelocity(new ChassisSpeeds(0, 0, m_lastRotationRateRequest));
                },
                log -> log.motor("rotation")
                    .voltage(Volts.of(m_lastRotationRateRequest))
                    .angularPosition(m_gyro.getYaw().getValue()) //pigeon yaw is continuous, doesn't wrap
                    .angularVelocity(m_gyro.getAngularVelocityZWorld().getValue()),
                this,
                "rotation"));

        SmartDashboard.putData("Field", m_field);
    }

    // ---------- driving ----------

    /** Robot relative. Also what path following should call. */
    public void runVelocity(ChassisSpeeds speeds) {
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, kLoopPeriodSecs);
        SwerveModuleState[] states = DriveConstants.kKinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeed);
        setModuleStates(states);
    }

    /** Field relative from the driver's point of view, forward is always away from your alliance wall. */
    public void driveFieldRelative(ChassisSpeeds fieldSpeeds) {
        runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, getDriverHeading()));
    }

    /**
     * Field relative translation (driver's point of view, same as driveFieldRelative) while the heading PID
     * points the robot at targetHeading. targetHeading is in the blue-origin field frame, so it's already
     * correct for things like aiming at a field position on either alliance.
     */
    public void driveFieldRelativeFacingAngle(double xMetersPerSec, double yMetersPerSec, Rotation2d targetHeading) {
        double omega = m_headingController.calculate(getPose().getRotation().getRadians(), targetHeading.getRadians());
        omega = Math.max(-DriveConstants.kMaxAngularSpeed.in(RadiansPerSecond),
            Math.min(DriveConstants.kMaxAngularSpeed.in(RadiansPerSecond), omega));
        driveFieldRelative(new ChassisSpeeds(xMetersPerSec, yMetersPerSec, omega));
    }

    public void stop() {
        runVelocity(new ChassisSpeeds()); //kinematics keeps the last module angles when speeds are all 0
    }

    /** Points the wheels in an X so the robot is hard to push. */
    public void xLock() {
        setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),   //fl
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),  //fr
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),  //bl
            new SwerveModuleState(0, Rotation2d.fromDegrees(45))    //br
        });
    }

    private void setModuleStates(SwerveModuleState[] states) {
        for (int i = 0; i < m_swerveModules.size(); i++) {
            m_swerveModules.get(i).setState(states[i]);
        }
        m_setpointStatesPub.set(states);
    }

    // ---------- pose / state ----------

    public Pose2d getPose() {
        return m_poser.getEstimatedPosition();
    }

    /** Pose is always in the blue-origin field frame, even on red. */
    public void resetPose(Pose2d pose) {
        m_poseSetExplicitly = true;
        m_poser.resetPosition(m_gyro.getRotation2d(), getModulePositions(), pose);
    }

    /** Makes the direction the robot is currently facing "forward" (away from the driver) for the driver. */
    public void zeroHeading() {
        resetPose(new Pose2d(getPose().getTranslation(), getAllianceForward()));
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        m_poser.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kKinematics.toChassisSpeeds(getModuleStates());
    }

    public SwerveModuleState[] getModuleStates() {
        return m_swerveModules.stream().map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
    }

    private SwerveModulePosition[] getModulePositions() {
        return m_swerveModules.stream().map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);
    }

    private Rotation2d getDriverHeading() {
        Rotation2d heading = getPose().getRotation();
        return isRedAlliance() ? heading.plus(Rotation2d.k180deg) : heading;
    }

    private boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }

    /** Field heading that points away from our driver station wall. */
    private Rotation2d getAllianceForward() {
        return isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero;
    }

    // ---------- sysid / characterization ----------

    //run each of these in its own robot code boot so each routine gets its own log file

    public Command sysIdDriveQuasistatic(SysIdRoutine.Direction direction) {
        return alignModules(Rotation2d.kZero).andThen(m_driveSysId.quasistatic(direction));
    }

    public Command sysIdDriveDynamic(SysIdRoutine.Direction direction) {
        return alignModules(Rotation2d.kZero).andThen(m_driveSysId.dynamic(direction));
    }

    public Command sysIdTurnQuasistatic(SysIdRoutine.Direction direction) {
        return m_turnSysId.quasistatic(direction);
    }

    public Command sysIdTurnDynamic(SysIdRoutine.Direction direction) {
        return m_turnSysId.dynamic(direction);
    }

    public Command sysIdRotationQuasistatic(SysIdRoutine.Direction direction) {
        return alignModulesForRotation().andThen(m_rotationSysId.quasistatic(direction));
    }

    public Command sysIdRotationDynamic(SysIdRoutine.Direction direction) {
        return alignModulesForRotation().andThen(m_rotationSysId.dynamic(direction));
    }

    private Command alignModules(Rotation2d angle) {
        return run(() -> m_swerveModules.forEach(module -> module.runDriveCharacterization(Volts.of(0), angle)))
            .withTimeout(1.0);
    }

    private Command alignModulesForRotation() {
        //kinematics points every module tangent when you only ask for rotation, then we just don't drive
        return run(() -> {
            SwerveModuleState[] states = DriveConstants.kKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 1));
            for (int i = 0; i < m_swerveModules.size(); i++) {
                m_swerveModules.get(i).runDriveCharacterization(Volts.of(0), states[i].angle);
            }
        }).withTimeout(1.0);
    }

    /**
     * 6328's wheel radius characterization. Put the robot on carpet, run it, let it spin a few full
     * rotations, then disable. Copy "Drive/WheelRadiusCharacterization/Radius Inches" into kWheelRadius.
     */
    public Command wheelRadiusCharacterization() {
        SlewRateLimiter limiter = new SlewRateLimiter(CharacterizationConstants.kWheelRadiusRampRate);
        WheelRadiusState state = new WheelRadiusState();

        return Commands.parallel(
            //spin in place, ramping up slowly so the wheels don't slip
            Commands.sequence(
                Commands.runOnce(() -> limiter.reset(0.0)),
                run(() -> runVelocity(new ChassisSpeeds(0, 0,
                    limiter.calculate(CharacterizationConstants.kWheelRadiusMaxVelocity))))),

            //measure
            Commands.sequence(
                Commands.waitSeconds(1.0), //let the modules get to their tangent angles first
                Commands.runOnce(() -> {
                    state.startWheelPositions = getWheelRadians();
                    state.lastAngle = m_gyro.getRotation2d();
                    state.gyroDelta = 0.0;
                }),
                Commands.run(() -> {
                    Rotation2d rotation = m_gyro.getRotation2d();
                    state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                    state.lastAngle = rotation;

                    double[] positions = getWheelRadians();
                    double wheelDelta = 0.0;
                    for (int i = 0; i < positions.length; i++) {
                        wheelDelta += Math.abs(positions[i] - state.startWheelPositions[i]) / positions.length;
                    }
                    double wheelRadius = (state.gyroDelta * kBodyMeasures.kDriveBaseRadius.in(Meters)) / wheelDelta;
                    state.radiusInches = Meters.of(wheelRadius).in(Inches);

                    SmartDashboard.putNumber("Drive/WheelRadiusCharacterization/Wheel Delta Rad", wheelDelta);
                    SmartDashboard.putNumber("Drive/WheelRadiusCharacterization/Gyro Delta Rad", state.gyroDelta);
                    SmartDashboard.putNumber("Drive/WheelRadiusCharacterization/Radius Meters", wheelRadius);
                    SmartDashboard.putNumber("Drive/WheelRadiusCharacterization/Radius Inches", state.radiusInches);
                }).finallyDo(() -> System.out.println(
                    "********** Wheel Radius Characterization **********\n"
                    + "  Radius: " + state.radiusInches + " inches")))
        );
    }

    /**
     * Finds the stator current where the drive wheels start slipping on carpet. Push the robot's FRONT bumper
     * flat against a wall and HOLD the button. Drive voltage ramps up on all four wheels (modules at 0deg)
     * until one wheel's velocity jumps (it broke loose), then all wheels stop. Let go of the button to end it
     * and put the normal current limit back. Set DriveMotorConstants.kCurrentLimit a little under the result.
     */
    public Command slipCurrentCharacterization() {
        SlipCurrentState state = new SlipCurrentState();

        return Commands.sequence(
            runOnce(() -> {
                state.reset();
                m_swerveModules.forEach(module -> module.getDriveMotor().setCurrentLimit(CharacterizationConstants.kSlipTestCurrentLimit));
                System.out.println("Slip test current limits: " + m_swerveModules.get(0).getDriveMotor().getCurrentLimitsString());
            }),
            alignModules(Rotation2d.kZero),
            runOnce(() -> {
                state.timer.restart();
                for (int i = 0; i < m_swerveModules.size(); i++) {
                    state.lastSpeed[i] = Math.abs(m_swerveModules.get(i).getDriveSpeed().in(MetersPerSecond));
                    state.jumpDebouncers[i].calculate(false);
                }
            }),
            run(() -> {
                if (state.slippedModule >= 0) {
                    //already found it, keep everything stopped until the button is released
                    m_swerveModules.forEach(module -> module.runDriveCharacterization(Volts.of(0), Rotation2d.kZero));
                    return;
                }

                double now = state.timer.get();
                double dt = now - state.lastTime;
                state.lastTime = now;
                state.volts = Math.min(now * CharacterizationConstants.kSlipRampRate, 12.0);

                for (int i = 0; i < m_swerveModules.size(); i++) {
                    SwerveModule module = m_swerveModules.get(i);
                    double speed = Math.abs(module.getDriveSpeed().in(MetersPerSecond));
                    double accel = dt > 0 ? (speed - state.lastSpeed[i]) / dt : 0.0;
                    state.lastSpeed[i] = speed;

                    boolean jumped = state.jumpDebouncers[i].calculate(accel > CharacterizationConstants.kSlipAccelThreshold);
                    if (!jumped) {
                        //current drops once the wheel spins up, so the peak while stalled is the real slip current
                        state.peakCurrent[i] = Math.max(state.peakCurrent[i], module.getDriveMotor().getCurrent().in(Amps));
                    } else if (state.slippedModule < 0) {
                        state.slippedModule = i;
                        state.slipVolts = state.volts;
                    }

                    SmartDashboard.putNumber("Drive/SlipCurrentCharacterization/" + module.getName() + " Peak Amps", state.peakCurrent[i]);
                    SmartDashboard.putNumber("Drive/SlipCurrentCharacterization/" + module.getName() + " Accel", accel);
                }
                SmartDashboard.putNumber("Drive/SlipCurrentCharacterization/Volts", state.volts);

                double outputVolts = state.slippedModule >= 0 ? 0.0 : state.volts;
                m_swerveModules.forEach(module -> module.runDriveCharacterization(Volts.of(outputVolts), Rotation2d.kZero));
            })
        ).finallyDo(() -> {
            m_swerveModules.forEach(module -> {
                module.runDriveCharacterization(Volts.of(0), Rotation2d.kZero);
                module.getDriveMotor().setCurrentLimit(DriveMotorConstants.kCurrentLimit);
            });

            StringBuilder results = new StringBuilder("********** Slip Current Characterization **********\n");
            if (state.slippedModule >= 0) {
                results.append("  ").append(m_swerveModules.get(state.slippedModule).getName())
                    .append(" slipped first at ").append(state.peakCurrent[state.slippedModule]).append("A (")
                    .append(state.slipVolts).append("V)\n");
            } else {
                results.append("  Nothing slipped before the button was released (got to ").append(state.volts).append("V)\n");
            }
            results.append("  Peak currents:");
            for (int i = 0; i < m_swerveModules.size(); i++) {
                results.append(" ").append(m_swerveModules.get(i).getName()).append("=").append(state.peakCurrent[i]).append("A");
            }
            System.out.println(results);
        });
    }

    private static class SlipCurrentState {
        final Timer timer = new Timer();
        final double[] peakCurrent = new double[4];
        final double[] lastSpeed = new double[4];
        final Debouncer[] jumpDebouncers = new Debouncer[4];
        double lastTime = 0.0;
        int slippedModule = -1;
        double volts = 0.0;
        double slipVolts = 0.0;

        void reset() {
            Arrays.fill(peakCurrent, 0.0);
            Arrays.fill(lastSpeed, 0.0);
            for (int i = 0; i < jumpDebouncers.length; i++) {
                jumpDebouncers[i] = new Debouncer(CharacterizationConstants.kSlipDebounceSecs, DebounceType.kRising);
            }
            lastTime = 0.0;
            slippedModule = -1;
            volts = 0.0;
            slipVolts = 0.0;
        }
    }

    private double[] getWheelRadians() {
        return m_swerveModules.stream().mapToDouble(SwerveModule::getWheelRadians).toArray();
    }

    private static class WheelRadiusState {
        double[] startWheelPositions = new double[4];
        Rotation2d lastAngle = Rotation2d.kZero;
        double gyroDelta = 0.0;
        double radiusInches = 0.0;
    }

    @Override
    public void periodic() {
        m_poser.update(m_gyro.getRotation2d(), getModulePositions());

        //alliance shows up once the DS connects (and can change in practice), so keep the heading matched while disabled
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (!m_poseSetExplicitly && DriverStation.isDisabled() && alliance.isPresent() && !alliance.equals(m_lastAlliance)) {
            m_lastAlliance = alliance;
            m_poser.resetPosition(m_gyro.getRotation2d(), getModulePositions(), new Pose2d(getPose().getTranslation(), getAllianceForward()));
        }

        m_swerveModules.forEach(SwerveModule::periodic);
        m_measuredStatesPub.set(getModuleStates());
        m_posePub.set(getPose());
        m_field.setRobotPose(getPose());
        SmartDashboard.putNumber("Drive/Gyro Deg", m_gyro.getRotation2d().getDegrees());
    }
}
