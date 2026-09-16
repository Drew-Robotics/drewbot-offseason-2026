package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

public class VisionConstants {
    public static final AprilTagFieldLayout kAprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static final class StandardDeviations {
        public static final Vector<N3> kSwerveArducamSingle = VecBuilder.fill(4, 4, 8); //yeah I copy pasted these values
        public static final Vector<N3> kSwerveArducamMulti = VecBuilder.fill(0.5, 0.5, 1);

        public static final Vector<N3> kDiscard = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }
}