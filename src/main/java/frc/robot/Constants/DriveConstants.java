package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class DriveConstants {
    public static final LinearVelocity maxSpeed = Units.FeetPerSecond.of(25);

    public static final class GyroscopeConstants {
        public static final int kCanID = 10;
    }

    public static final class TurnMotorConstants{
        public static final int kCurrentLimit = 20;

        public static final Angle kPositionConversionFactor = Units.Radians.of(2*Math.PI/5);
        public static final AngularVelocity kVelocityConversionFactor = Units.RotationsPerSecond.of(2*Math.PI/5);

        public static final class PID {
            public static final double kP = 100;
            public static final double kI = 0;
            public static final double kD = 0;
        }
    }

    public static final class DriveMotorConstants{
        public static final int kCurrentLimit = 40;

        public static final Distance kPositionConversionFactor = Units.Meters.of(0.05678);
        //native velocity is RPM, so it's the position factor per minute
        public static final LinearVelocity kVelocityConversionFactor = Units.MetersPerSecond.of(0.05678/60);

        public static final class PID {
            public static final double kP = 0.1;
            public static final double kI = 0;
            public static final double kD = 0;
        }
    }


    public static final class FrontLeftModule {
        public static final int kDriveCANID = 5;
        public static final boolean kDriveInverted = false;

        public static final int kTurnCANID = 6;
        public static final boolean kTurnInverted = false;

        public static final Rotation2d kOffset = Rotation2d.fromRadians(3.595);
    }
    public static final class FrontRightModule {
        public static final int kDriveCANID = 3;
        public static final boolean kDriveInverted = true;

        public static final int kTurnCANID = 4;
        public static final boolean kTurnInverted = false;

        public static final Rotation2d kOffset = Rotation2d.fromRadians(4.955);
    }

    public static final class BackLeftModule {
        public static final int kDriveCANID = 7;
        public static final boolean kDriveInverted = false;

        public static final int kTurnCANID = 8;
        public static final boolean kTurnInverted = false;

        public static final Rotation2d kOffset = Rotation2d.fromRadians(1.035);
    }

    public static final class BackRightModule {
        public static final int kDriveCANID = 1;
        public static final boolean kDriveInverted = true;

        public static final int kTurnCANID = 2;
        public static final boolean kTurnInverted = false;

        public static final Rotation2d kOffset = Rotation2d.fromRadians(3.114);
    }

    public static final class kBodyMeasures{
        public static final Distance kWheelBase = Units.Inches.of(21.25);
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
