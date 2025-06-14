package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class Camera {

  private PhotonCamera camera;
  private PhotonPoseEstimator poseEstimator;
  private Pose3d lastVisionPose;
  private double lastPoseUpdate;

  public Camera(
    String cameraName,
    AprilTagFieldLayout field,
    Transform3d robotToCameraPose
  ) {
    this.camera = new PhotonCamera(cameraName);
    this.poseEstimator = new PhotonPoseEstimator(
      field,
      PoseStrategy.LOWEST_AMBIGUITY,
      robotToCameraPose
    );
  }

  public double getLatencyTime() {
    return this.lastPoseUpdate;
  }

  // run once every robot frame or else no workie
  public Optional<Pose2d> getEstimatedRobotPose(Pose2d drivetrainPose) {
    final List<PhotonPipelineResult> pipelineResults =
      camera.getAllUnreadResults();

    if (pipelineResults.size() <= 0) {
      return Optional.empty();
    }

    Optional<PhotonPipelineResult> optPipeline = Optional.of(
      pipelineResults.get(0)
    ); // optional because index can be null if photonvision disconnects
    // depends on latest pipeline result
    if (
      optPipeline.isEmpty() ||
      optPipeline.get().hasTargets() == false ||
      optPipeline.get().getBestTarget().poseAmbiguity > 0.2
    ) {
      return Optional.empty();
    }

    PhotonPipelineResult pipeline = optPipeline.get();

    if (lastVisionPose != null) poseEstimator.setReferencePose(lastVisionPose);
    final Optional<EstimatedRobotPose> possiblePose = poseEstimator.update(
      pipeline
    );

    if (possiblePose.isEmpty()) {
      return Optional.empty();
    }
    final Pose2d possible2dPose = possiblePose.get().estimatedPose.toPose2d();
    this.lastVisionPose = possiblePose.get().estimatedPose;
    this.lastPoseUpdate = possiblePose.get().timestampSeconds;
    return Optional.of(
      possible2dPose.transformBy(
        new Transform2d(
          new Translation2d(0,0),
          new Rotation2d(Math.toRadians(180))
        )
      )
    );
  }
}
