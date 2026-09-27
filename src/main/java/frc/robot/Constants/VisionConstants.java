package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class VisionConstants {
    public static final AprilTagFieldLayout kAprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static final class StandardDeviations {
        public static final Vector<N3> kSwerveArducamSingle = VecBuilder.fill(4, 4, 8); //yeah I copy pasted these values
        public static final Vector<N3> kSwerveArducamMulti = VecBuilder.fill(0.5, 0.5, 1);

        public static final Vector<N3> kDiscard = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }

    public static final class cameraTransformations {
        public static final class wheelToCamera {
            public static final Distance x = Units.Inches.of(2.202);
            public static final Distance y = Units.Inches.of(2.200);
            public static final Distance z = Units.Inches.of(5.111);
        }

        public static final Angle kPitch = Units.Degrees.of(26.0);

        public static final Angle kFRYaw = Units.Degrees.of(45);
        public static final Angle kBLYaw = Units.Degrees.of(225);

        public static final Angle kRoll = Units.Degrees.of(0);


        public static final Transform3d kFRTransformation = new Transform3d(
            new Translation3d(
                DriveConstants.kBodyMeasures.kWheelBase.plus(wheelToCamera.x),
                DriveConstants.kBodyMeasures.kWheelBase.plus(wheelToCamera.y),
                wheelToCamera.z
            ),
            new Rotation3d(kRoll.in(Units.Degrees), kPitch.in(Units.Degrees), kFRYaw.in(Units.Degrees))
        );

        public static final Transform3d kBLTransformation = new Transform3d(
            new Translation3d(
                DriveConstants.kBodyMeasures.kWheelBase.plus(wheelToCamera.x).times(-1),
                DriveConstants.kBodyMeasures.kWheelBase.plus(wheelToCamera.y).times(-1),
                wheelToCamera.z
            ),
            new Rotation3d(kRoll.in(Units.Degrees), kPitch.in(Units.Degrees), kBLYaw.in(Units.Degrees))
        );
        public static final Transform3d kBackLeftTransformation = new Transform3d();
    }
}