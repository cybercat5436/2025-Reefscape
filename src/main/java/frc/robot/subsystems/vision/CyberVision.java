package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;

public record CyberVision(
    int cameraIndex,
    double timestamp,
    Translation2d poseTranslation2d,
    double poseRotationDegrees,
    double ambiguity,
    int tagCount,
    double averageTagDistance,
    PoseObservationType type,
    double distanceFromRobotPose, // error between poseEstimate and visionEstimate
    double translationStdDev,
    double rotationStdDev,
    boolean accepted,
    int tagIds
) {

}
