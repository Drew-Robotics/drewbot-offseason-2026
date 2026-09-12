package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.AnalogInput;

public class DriveConstants {
    public static final LinearVelocity maxSpeed = Units.MetersPerSecond.of(10);

    public static final class GyroscopeConstants {
        public static final int kCanID = 0;
    }

    public static final class EncoderConstants {
        public static final Angle EncoderConversion = Units.Radians.of(0); //converting from volts to radians
    }

    public static final class TurnMotorConstants{
        public static final int kCurrentLimit = 20;

        public static final Angle kPositionConversionFactor = Units.Radians.of(0.24166); //radians pls queen
        public static final AngularVelocity kVelocityConversionFactor = Units.RadiansPerSecond.of(0.24166);

        public static final class PID {
            public static final double kP = 0;
            public static final double kI = 0;
            public static final double kD = 0;
        }
    }

    public static final class DriveMotorConstants{
        public static final int kCurrentLimit = 40;

        public static final Distance kPositionConversionFactor = Units.Meters.of(0.05678); //these ts variables need to be meters pls, stop being lazy you freaking neanderthal
        public static final LinearVelocity kVelocityConversionFactor = Units.MetersPerSecond.of(0.05678);

        public static final class PID {
            public static final double kP = 0;
            public static final double kI = 0;
            public static final double kD = 0;
        }
    }


    public static final class FrontLeftModule {
        public static final int kDriveCANID = 5;
        public static final AnalogInput kAnalogInput = new AnalogInput(1);
        public static final boolean kDriveInverted = false;

        public static final int kTurnCANID = 6;
        public static final boolean kTurnInverted = false;

        public static final double kOffset = 2.845;
    }
    public static final class FrontRightModule {
        public static final int kDriveCANID = 3;
        public static final AnalogInput kAnalogInput = new AnalogInput(3);
        public static final boolean kDriveInverted = false;

        public static final int kTurnCANID = 4;
        public static final boolean kTurnInverted = false;

        public static final double kOffset = 3.865;
    }
    public static final class BackRightModule {
        public static final int kDriveCANID = 1;
        public static final AnalogInput kAnalogInput = new AnalogInput(2);
        public static final boolean kDriveInverted = false;

        public static final int kTurnCANID = 2;
        public static final boolean kTurnInverted = false;

        public static final double kOffset = 2.4927;
    }
    public static final class BackLeftModule {
        public static final int kDriveCANID = 7;
        public static final AnalogInput kAnalogInput = new AnalogInput(0);
        public static final boolean kDriveInverted = false;

        public static final int kTurnCANID = 8;
        public static final boolean kTurnInverted = false;

        public static final double kOffset = 0.842;
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
