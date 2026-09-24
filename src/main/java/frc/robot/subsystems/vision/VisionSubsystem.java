package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DriveSubsystem;

public class VisionSubsystem extends SubsystemBase{
    private final List<Camera> m_cameras;

    private static VisionSubsystem m_instance;

    public VisionSubsystem() {
        m_cameras = List.of();
    }

    public static VisionSubsystem getInstance(){
        if(m_instance==null) {
            m_instance = new VisionSubsystem();
        }
        return m_instance;
    }

    public List<AprilTag> seenAprilTags() {
        List<AprilTag> tags = List.of();
        for(Camera cam : m_cameras){
            for(AprilTag tag : cam.getSeenAprilTags()){
                if(tags.contains(tag)==false) {
                    tags.add(tag);
        }   }   }
        return tags;
    }

    public Optional<AprilTag> closestAprilTag() {
        if(seenAprilTags().size()==0) {
            return Optional.empty();
        }

        List<AprilTag> seenTags = seenAprilTags();
        AprilTag closestTag = seenTags.get(0);
        for(AprilTag tag : seenTags){
            if(
                distanceToTag(tag, DriveSubsystem.getInstance().getPose()).in(Units.Meters)
                <distanceToTag(closestTag, DriveSubsystem.getInstance().getPose()).in(Units.Meters)){
                    closestTag=tag;
        }   }   
        return Optional.of(closestTag);
    }

    public static Distance distanceToTag (AprilTag tag, Pose2d robotPose){
        return Units.Meters.of(tag.pose
            .getTranslation()
            .toTranslation2d()
            .getDistance(robotPose.getTranslation()));
    }

    // public List<EstimatedRobotPose> getPoseEstimations () {
    //     return m_cameras.stream()
    //         .map(c -> c.getPoseEstimation().get())
    //         .toList();
    // }

    public List<Camera> getCameras () {
        return m_cameras;
    }
}