package frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class DriveConstants {
    public static final LinearVelocity maxSpeed = Units.MetersPerSecond.of(0);

    public static final class Gyroscope{
        public static final int kCanID = 0;
    }

    public static final class TurnMotorConstants{
        public static final int kCurrentLimit = 0;

        public static final Angle kPositionConversionFactor = Units.Radians.of(0); //radians pls queen
        public static final AngularVelocity kVelocityConversionFactor = Units.RadiansPerSecond.of(0);

        public static final class PID {
            public static final double kP = 0;
            public static final double kI = 0;
            public static final double kD = 0;
        }
    }

    public static final class DriveMotorConstants{
        public static final int kCurrentLimit = 0;

        public static final Distance kPositionConversionFactor = Units.Meters.of(0); //these ts variables need to be meters pls, stop being lazy you freaking troglodyte
        public static final LinearVelocity kVelocityConversionFactor = Units.MetersPerSecond.of(0);

        public static final class PID {
            public static final double kP = 0;
            public static final double kI = 0;
            public static final double kD = 0;
        }
    }


    public static final class FrontLeftModule {
        public static final int kDriveCANID = 0;
        public static final boolean kDriveInverted = false;

        public static final int kTurnCANID = 0;
        public static final boolean kTurnInverted = false;

        public static final Rotation2d kOffset = Rotation2d.fromDegrees(0);
    }
    public static final class FrontRightModule {
        public static final int kDriveCANID = 0;
        public static final boolean kDriveInverted = false;

        public static final int kTurnCANID = 0;
        public static final boolean kTurnInverted = false;

        public static final Rotation2d kOffset = Rotation2d.fromDegrees(0);
    }
    public static final class BackRightModule {
        public static final int kDriveCANID = 0;
        public static final boolean kDriveInverted = false;

        public static final int kTurnCANID = 0;
        public static final boolean kTurnInverted = false;

        public static final Rotation2d kOffset = Rotation2d.fromDegrees(0);
    }
    public static final class BackLeftModule {
        public static final int kDriveCANID = 0;
        public static final boolean kDriveInverted = false;

        public static final int kTurnCANID = 0;
        public static final boolean kTurnInverted = false;

        public static final Rotation2d kOffset = Rotation2d.fromDegrees(0);
    }

    public static final class kBodyMeasures{
        public static final Distance kWheelBase = Units.Inches.of(0);
    }

    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
      new Translation2d(
        kBodyMeasures.kWheelBase.div(2), 
        kBodyMeasures.kWheelBase.div(2)),//fl
      new Translation2d(
          kBodyMeasures.kWheelBase.div(2), 
          kBodyMeasures.kWheelBase.div(2).times(-1)),//fr
      new Translation2d(
          kBodyMeasures.kWheelBase.div(2).times(-1), 
          kBodyMeasures.kWheelBase.div(2)),//bl
      new Translation2d(
          kBodyMeasures.kWheelBase.div(2).times(-1),
          kBodyMeasures.kWheelBase.div(2).times(-1)));//br
}
