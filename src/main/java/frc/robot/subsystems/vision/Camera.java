package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.opencv.core.Mat.Tuple2;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.VisionConstants.StandardDeviations;

public class Camera {
    private final PhotonCamera m_camera;
    private final PhotonPoseEstimator m_poser;

    private Optional<PhotonPipelineResult> m_latestResult;
    private Optional<EstimatedRobotPose> m_latestPoseEstimation;
    private Optional<Vector<N3>> m_standardDeviation;

    public Camera(String name, Transform3d transformToCamera, AprilTagFieldLayout field) {
        m_camera = new PhotonCamera(name);
        m_poser = new PhotonPoseEstimator(field, transformToCamera);

        m_latestPoseEstimation = Optional.empty();
        m_latestResult = Optional.empty();
        m_standardDeviation = Optional.empty();
    }

    private void updateLatestResults() {
        // m_latestResult = m_camera.getAllUnreadResults().stream().findFirst();
        m_latestResult = Optional.of(m_camera.getAllUnreadResults().get(0));
    }

    private void updatePose() {
        m_latestPoseEstimation =
            m_latestResult.isPresent() ? m_poser.estimateLowestAmbiguityPose(m_latestResult.get()) : Optional.empty();
    }

    private Optional<Vector<N3>> getStandardDeviations (Pose2d robotPose) {
        List<Double> distances = getSeenAprilTags().stream().map(tag -> distanceToTag(tag, robotPose).in(Units.Meters)).toList();

        if(distances.isEmpty()){
            return Optional.empty();
        }

        if(distances.size()==1){
            double distance = distances.get(0);
            return distance > 4 ? Optional.of(StandardDeviations.kDiscard) : Optional.of(StandardDeviations.kSwerveArducamSingle.times(1 + distance*distance/100));
        }

        Double averageDistance = 0.0;
        for(Double distance : distances) {averageDistance+=distance;} //first time using a for loop for frc btw
        averageDistance/=distances.size(); //there's some way to do this in one line with a stream but I'm too lazy to do that

        return Optional.of(StandardDeviations.kSwerveArducamMulti.times(1+averageDistance*averageDistance/100));
    }

    private List<PhotonTrackedTarget> getSeenTargets (Optional<PhotonPipelineResult> result){
        return result.isPresent() ? result.get().getTargets() : List.of();
    }

    public List<AprilTag> getSeenAprilTags () {
        return getSeenTargets(m_latestResult).stream()
            .map (target -> target.getFiducialId())
            .filter(id -> id!=-1)
            .map(id -> new AprilTag(id, VisionConstants.kAprilTagLayout.getTagPose(id).orElse(new Pose3d())))
            .toList();
    }

    public Distance distanceToTag (AprilTag tag, Pose2d robotPose){
        return Units.Meters.of(tag.pose
            .getTranslation()
            .toTranslation2d()
            .getDistance(robotPose.getTranslation())); //MAYBE SHOULDN'T BE METERS IDK I'M LOWK GUESSING    hindsight I'm like 85% sure I was right but still leaving this here
    }

    public Optional<EstimatedRobotPose> getPoseEstimation () {
        Optional<EstimatedRobotPose> p = m_latestPoseEstimation;
        m_latestPoseEstimation = Optional.empty();
        return p;
    }
    //next we need getPoseEstimation and the little scraps at the end to make it a complete function

}//to any freshmen reading this file I apologize greatly for my comment standards