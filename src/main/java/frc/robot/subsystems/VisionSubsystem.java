package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionSubsystem extends SubsystemBase {

  private String warning = "";
  private Pose3d lastVisionPose;

  private PhotonCamera frontCamera = new PhotonCamera("front");
  private AprilTagFieldLayout aprilTagFieldLayout =
    AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  private Transform3d robotToFrontCam = new Transform3d(
    new Translation3d(0, 0.14, 0.5),
    new Rotation3d(0, Math.toRadians(0), Math.toRadians(0))
  );
  private PhotonPoseEstimator photonPoseEstimator;
  private Pose2d estimatedRobotPose;
  private Optional<PhotonPipelineResult> latestPipelineResult;
  public double lastPoseUpdate = 0.0; // estimated time the frame was taken, in the Time Sync Server's time base (nt::Now). Currently unused, but should be used lmao
  private SendableChooser<Mode> chooser = new SendableChooser<>();

  private Supplier<Pose2d> drivetrain;
  private SequentialCommandGroup cc = new SequentialCommandGroup();

  public enum Mode {
    REEF,
    PROCESSOR,
    BARGE,
  }

  public VisionSubsystem() {
    photonPoseEstimator = new PhotonPoseEstimator(
      aprilTagFieldLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      robotToFrontCam
    );
  }

  public void setDrivetrain(Supplier<Pose2d> dr) {
    drivetrain = dr;
  }

  @Override
  public void periodic() {
    if (!frontCamera.isConnected()) {
      estimatedRobotPose = null;
      return;
    }

    final List<PhotonPipelineResult> pipelineResults =
      frontCamera.getAllUnreadResults();

    if (pipelineResults.size() > 0) {
      latestPipelineResult = Optional.of(pipelineResults.get(0)); // optional because index can be null if photonvision disconnects
      // depends on latest pipeline result
      if (latestPipelineResult.isPresent()) {
        updateEstimatedRobotPose(latestPipelineResult.get());
      } else {
        estimatedRobotPose = null;
      }
    } else {
      estimatedRobotPose = null;
    }
  }

  /*
   * Stops the Vision system, preventing it from overriding swerve
   */
  public void stop() {
    cc.cancel();
  }

  /*
   * Updates the estimated robot pose for vision.
   */
  private void updateEstimatedRobotPose(PhotonPipelineResult pipeline) {
    if (lastVisionPose != null) photonPoseEstimator.setReferencePose(
      lastVisionPose
    );
    final Optional<EstimatedRobotPose> possiblePose =
      photonPoseEstimator.update(pipeline);

    if (possiblePose.isEmpty()) {
      this.estimatedRobotPose = null;
      return;
    }
    final Pose2d possible2dPose = possiblePose.get().estimatedPose.toPose2d();
    this.estimatedRobotPose = new Pose2d(
      possible2dPose.getX(),
      possible2dPose.getY(),
      drivetrain.get().getRotation()
    );
    this.lastVisionPose = possiblePose.get().estimatedPose;
    this.lastPoseUpdate = possiblePose.get().timestampSeconds;
  }

  /*
   * Get the estimated vision robot pose as 2d
   */
  public Pose2d getEstimatedPose2d() {
    return estimatedRobotPose;
  }

  public Optional<Integer> getClosestTagId(int[] tagList) {
    if (latestPipelineResult.isEmpty()) {
      warning = "NO LATEST PIPELINE";
      return Optional.empty();
    }

    if (drivetrain.get() == null) {
      warning = "NO ROBOT POSE";
      return Optional.empty();
    }

    int closestTag = -1;
    double closestDist = Double.MAX_VALUE;
    for (int tagId : tagList) {
      Pose3d tagPose = this.aprilTagFieldLayout.getTagPose(tagId).get();

      Pose2d dist = tagPose.toPose2d().relativeTo(drivetrain.get());
      double xDist = dist.getMeasureX().magnitude();
      double yDist = dist.getMeasureY().magnitude();

      double totalDistance = Math.sqrt(Math.pow(xDist, 2) + Math.pow(yDist, 2));
      if (totalDistance < closestDist) {
        closestDist = totalDistance;
        closestTag = tagId;
      }
    }

    return Optional.of(closestTag);
  }

  public Mode getMode() {
    return chooser.getSelected();
  }

  public Pose2d getTagPose2d(int tagId) {
    return this.aprilTagFieldLayout.getTagPose(tagId).get().toPose2d();
  }

  public boolean getIsL2FromTag(int tagId) {
    return Vision.reefAlgaeMap.get(tagId);
  }

  public boolean withinBarge(double robotYFromTag) {
    return Math.abs(robotYFromTag) < Vision.bargeLength / 2.0;
  }

  /*
   * Aligns to the nearest reef tag.
   */
  @Deprecated
  public void reefAlign() {
    Optional<Integer> closestTagOptional = getClosestTagId(Vision.reefIds);
    if (closestTagOptional.isEmpty()) return;

    int closestTag = closestTagOptional.get();
    final Pose2d desiredTag2d = getTagPose2d(closestTag).plus(
      new Transform2d(0.1, 0.0, new Rotation2d())
    );

    Command p = AutoBuilder.pathfindToPose(
      desiredTag2d,
      new PathConstraints(
        1.5,
        2,
        Units.degreesToRadians(3000),
        Units.degreesToRadians(2000)
      ),
      0.0
    ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    // patented D.N.K.R G.A.V.I.N (Do Not Kill Robot - General Autonomous Vision Information Networking)
    Command in = AutoBuilder.pathfindToPose(
      desiredTag2d.plus(new Transform2d(0.6, 0.0, new Rotation2d())),
      new PathConstraints(
        1,
        2,
        Units.degreesToRadians(3000),
        Units.degreesToRadians(2000)
      ),
      0.0
    ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

    cc.cancel();
    cc = new SequentialCommandGroup(in, p);
    cc.schedule();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Vision");

    chooser.setDefaultOption("Reef", Mode.REEF);
    chooser.addOption("Barge", Mode.BARGE);
    chooser.addOption("Processor", Mode.PROCESSOR);

    SmartDashboard.putData("Vision Mode", chooser);

    builder.addStringProperty("Last Warning", () -> warning, null);
    builder.addStringProperty(
      "Robot Pose (Vision)",
      () -> {
        if (estimatedRobotPose == null) return "";
        return estimatedRobotPose.toString();
      },
      null
    );
  }
}
