package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision implements Subsystem {

  public double robotRotationOverride = 0;

  PhotonCamera camera;
  AprilTagFieldLayout aprilTagFieldLayout;
  Transform3d robotToCam;
  PhotonPoseEstimator photonPoseEstimator;

  public Vision() {
    this.camera = new PhotonCamera("photonvision"); // temp name
    this.aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    this.robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
    this.PhotonPoseEstimator =
        new PhotonPoseEstimator(
            this.aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, this.camera, this.robotToCam);
  }

  public void bargeAlign() {
    final PhotonPipelineResult pipeline = camera.getAllUnreadResults().get(0);
    if (!pipeline.hasTargets()) return; // no targets found, obviously

    final PhotonTrackedTarget bestTarget = pipeline.getBestTarget();

    final Optional<EstimatedRobotPose> pose = photonPoseEstimator.update(pipeline);
    if (!pose.isPresent()) return; // no pose, obviously

    final Pose3d robotPose = pose.get().estimatedPose;
    final Pose3d desiredTag = this.aprilTagFieldLayout.getTagPose(bestTarget.fiducialId).get();
    final Translation3d diff = desiredTag.getTranslation().minus(robotPose.getTranslation());

    double yaw = Math.atan2(diff.getX(), diff.getY());

    this.robotRotationOverride = yaw;
  }
}
