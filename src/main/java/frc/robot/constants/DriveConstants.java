package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class DriveConstants {

    public static final class GyroscopeConstants {
        public static final int kCanID = 10; //verified in Tuner X
    }

    public static final class kBodyMeasures {
        //center of wheel to center of wheel. these MUST be nonzero: kDriveBaseRadius divides by them
        public static final Distance kWheelBase = Units.Inches.of(21.25);  //front to back
        public static final Distance kTrackWidth = Units.Inches.of(21.25); //left to right, square chassis

        //nominal 4in, replace with the output of the wheel radius characterization
        public static final Distance kWheelRadius = Units.Inches.of(2.0);

        //distance from robot center to a module, used for rotation speed + wheel radius characterization
        public static final Distance kDriveBaseRadius = Units.Meters.of(
            Math.hypot(kWheelBase.in(Units.Meters) / 2, kTrackWidth.in(Units.Meters) / 2));
    }

    public static final class TurnMotorConstants {
        public static final int kCurrentLimit = 20;
        public static final IdleMode kIdleMode = IdleMode.kBrake;

        public static final double kGearRatio = 26.0; //MK5i steering is 26:1

        //thrifty encoder on the flex data port reports 0-5V for one full module rotation
        public static final double kAnalogMaxVolts = 5.0;
        //flip this if positive turn voltage makes the encoder angle go DOWN (should be CCW positive looking from above)
        public static final boolean kEncoderInverted = false;

        //analog sensor volts -> module radians, only used to seed the internal encoder
        public static final double kAnalogPositionConversionFactor = 2 * Math.PI / kAnalogMaxVolts;
        public static final double kAnalogVelocityConversionFactor = 2 * Math.PI / kAnalogMaxVolts; //analog velocity is V/s

        //vortex internal encoder rotations -> module radians. closed loop runs on this, seeded from the analog sensor
        public static final double kMotorPositionConversionFactor = 2 * Math.PI / kGearRatio;
        public static final double kMotorVelocityConversionFactor = kMotorPositionConversionFactor / 60.0;

        public static final class PID { //from turn sysid (position loop, radians)
            //placeholder from studentDriver's working loop. that loop ran on the analog sensor but its
            //feedback was also module radians, so kP carries over. stiff (~0.6deg error saturates output).
            //TODO replace with the real turn sysid result
            public static final double kP = 100;
            public static final double kI = 0;
            public static final double kD = 0;
        }
    }

    public static final class DriveMotorConstants {
        public static final int kCurrentLimit = 40;
        public static final IdleMode kIdleMode = IdleMode.kBrake;

        public static final double kGearRatio = 5.27; //MK5i R3, TODO verify against the SDS drawing
        public static final DCMotor kMotor = DCMotor.getNeoVortex(1);

        //motor rotations -> meters of wheel travel
        public static final double kPositionConversionFactor = 2 * Math.PI * kBodyMeasures.kWheelRadius.in(Units.Meters) / kGearRatio;
        public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;

        public static final class PID { //from drive sysid (velocity loop, meters)
            public static final double kP = 0;
            public static final double kI = 0;
            public static final double kD = 0;
        }

        public static final class Feedforward { //from drive sysid, volts / volts per m/s
            public static final double kS = 0;
            //stopgap so setVelocity does something before drive sysid runs: 12V / ~6.85 m/s free speed.
            //TODO replace with the real drive sysid result
            public static final double kV = 1.75;
        }
    }

    public static final class HeadingConstants {
        public static final class PID { //from rotation sysid (position loop), output is rad/s
            public static final double kP = 0;
            public static final double kI = 0;
            public static final double kD = 0;
        }
    }

    public static final class CharacterizationConstants {
        //6328 wheel radius characterization
        public static final double kWheelRadiusMaxVelocity = 0.25; //rad/s
        public static final double kWheelRadiusRampRate = 0.05;   //rad/s^2

        //turn sysid only spins one module's turn motor. 0=FL 1=FR 2=BL 3=BR, pick the matching URCL device in sysid
        public static final int kTurnSysIdModuleIndex = 0;

        //slip current characterization, robot's front bumper pushed against a wall
        public static final int kSlipTestCurrentLimit = 120;         //amps, temporarily raised so the limit isn't what stops the wheel
        public static final double kSlipRampRate = 0.2;              //volts/s
        //a stalled wheel barely moves, a slipping one spins up hard. slip = wheel acceleration above this...
        public static final double kSlipAccelThreshold = 2.0;        //m/s^2
        //...for this long, so one noisy encoder sample doesn't count
        public static final double kSlipDebounceSecs = 0.04;
    }

    public static final LinearVelocity kMaxSpeed = Units.MetersPerSecond.of(
        Units.RadiansPerSecond.of(DriveMotorConstants.kMotor.freeSpeedRadPerSec).in(Units.RPM)
            * DriveMotorConstants.kVelocityConversionFactor); //free speed, ~6.8 m/s
    public static final AngularVelocity kMaxAngularSpeed = Units.RadiansPerSecond.of(
        kMaxSpeed.in(Units.MetersPerSecond) / kBodyMeasures.kDriveBaseRadius.in(Units.Meters));

    //what full right stick asks for in teleop. the kinematic max above is ~2.9 rev/s, which is way more
    //than a driver can use. 1 rev/s is what studentDriver ran. purely feel, turn it up if they want it.
    public static final AngularVelocity kTeleopMaxAngularSpeed = Units.RadiansPerSecond.of(2 * Math.PI);

    //module offsets: point the wheel straight forward (bevel gears all facing the same side) and copy
    //Drive/<module>/Raw Absolute Angle Deg from the dashboard
    public static final class FrontLeftModule {
        public static final int kDriveCANID = 5;
        public static final boolean kDriveInverted = false;

        public static final int kTurnCANID = 6;
        public static final boolean kTurnInverted = false;

        public static final Rotation2d kOffset = Rotation2d.fromDegrees(205.978); //re-zeroed on studentDriver, 3.595 rad
    }
    public static final class FrontRightModule {
        public static final int kDriveCANID = 3;
        public static final boolean kDriveInverted = true; //right side modules are mirrored

        public static final int kTurnCANID = 4;
        public static final boolean kTurnInverted = false;

        public static final Rotation2d kOffset = Rotation2d.fromDegrees(283.901); //re-zeroed on studentDriver, 4.955 rad
    }
    public static final class BackLeftModule {
        public static final int kDriveCANID = 7;
        public static final boolean kDriveInverted = false;

        public static final int kTurnCANID = 8;
        public static final boolean kTurnInverted = false;

        public static final Rotation2d kOffset = Rotation2d.fromDegrees(59.301); //re-zeroed on studentDriver, 1.035 rad
    }
    public static final class BackRightModule {
        public static final int kDriveCANID = 1;
        public static final boolean kDriveInverted = true; //right side modules are mirrored

        public static final int kTurnCANID = 2;
        public static final boolean kTurnInverted = false;

        public static final Rotation2d kOffset = Rotation2d.fromDegrees(178.419); //re-zeroed on studentDriver, 3.114 rad
    }

    //order everywhere is FL, FR, BL, BR
    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
        new Translation2d(kBodyMeasures.kWheelBase.div(2), kBodyMeasures.kTrackWidth.div(2)),         //fl
        new Translation2d(kBodyMeasures.kWheelBase.div(2), kBodyMeasures.kTrackWidth.div(-2)),        //fr
        new Translation2d(kBodyMeasures.kWheelBase.div(-2), kBodyMeasures.kTrackWidth.div(2)),        //bl
        new Translation2d(kBodyMeasures.kWheelBase.div(-2), kBodyMeasures.kTrackWidth.div(-2)));      //br
}
