package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {

  private PIDController xPID = new PIDController(1.2, 0, 0);
  private PIDController yPID = new PIDController(1.2, 0, 0);

  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 3/4 of a rotation per second max
  // angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentricFacingAngle drive =
    new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(MaxSpeed * 0.1)
      .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.Velocity);

  private Rotation2d rotation = new Rotation2d();
  private double moveX = 0.0;
  private double moveY = 0.0;
  private boolean overrideSwerve = false;

  private PhotonCamera frontCamera = new PhotonCamera("front");
  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  private Transform3d robotToFrontCam;
  private PhotonPoseEstimator photonPoseEstimator;
  private EstimatedRobotPose estimatedRobotPose;
  private Optional<PhotonPipelineResult> latestPipelineResult;

  private static int[] reefIds = new int[] {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};

  public VisionSubsystem() {
    robotToFrontCam =
        new Transform3d(
            new Translation3d(0, 0.0, 0.5),
            new Rotation3d(0, Math.toRadians(30), Math.toRadians(180)));
    photonPoseEstimator =
        new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, robotToFrontCam);

    xPID.setSetpoint(1.5);
    yPID.setSetpoint(0);
    xPID.setTolerance(0.05);
    yPID.setTolerance(0.05);
  }

  @Override
  public void periodic() {
    final List<PhotonPipelineResult> pipelineResults = frontCamera.getAllUnreadResults();

    if (pipelineResults.size() > 0) {
      latestPipelineResult = Optional.of(pipelineResults.get(0)); // optional because index can be null if photonvision disconnects
      
      if (latestPipelineResult.isPresent()) updateEstimatedRobotPose(latestPipelineResult.get()); // depends on latest pipeline result
    }
  }

  /*
   * The override, returning a FieldCentric command
   */
  public FieldCentricFacingAngle drive() {
    return drive
        .withVelocityX(moveX) // For now, do not allow movement
        .withVelocityY(moveY) // For now, do not allow movement
        .withTargetDirection(this.rotation)
        .withHeadingPID(12, 0, 0);
  }

  /*
   * Check to see if vision is active and overriding swerve
   */
  public boolean isOverridingSwerve() {
    return overrideSwerve;
  }

  /*
   * Stops the Vision system, preventing it from overriding swerve
   */
  public void stop() {
    overrideSwerve = false;
  }

  /*
   * Updates the estimated robot pose for vision.
   */
  private void updateEstimatedRobotPose(PhotonPipelineResult pipeline) {
    final Optional<EstimatedRobotPose> possiblePose = photonPoseEstimator.update(pipeline);

    if (possiblePose.isEmpty()) return;
    this.estimatedRobotPose = possiblePose.get();
  }

  /*
   * Get the estimated vision robot pose as 2d
   */
  public Pose2d getEstimatedPose2d() {
    return estimatedRobotPose.estimatedPose.toPose2d();
  }

  /*
   * Aligns to the nearest reef tag.
   */
  public void reefAlign() {

    if (latestPipelineResult.isEmpty()) {
      overrideSwerve = false;
      return;
    }

    // final PhotonTrackedTarget bestTarget = pipeline.get().getBestTarget();
    final List<PhotonTrackedTarget> targets = latestPipelineResult.get().getTargets();
    if (estimatedRobotPose == null || targets.isEmpty()) {
      overrideSwerve = false;
      return;
    }
    // no pose, obviously

    int closestTag = -1;
    double closestDist = Double.MAX_VALUE;
    for (PhotonTrackedTarget target : targets) {
      for (int tagId : reefIds) {
        if (tagId != target.getFiducialId()) {
          continue;
        }

        Pose3d tagPose = this.aprilTagFieldLayout.getTagPose(tagId).get();

        Transform3d dist = estimatedRobotPose.estimatedPose.minus(tagPose);
        double xDist = dist.getMeasureX().magnitude();
        double yDist = dist.getMeasureY().magnitude();

        double totalDistance = Math.sqrt(Math.pow(xDist, 2) + Math.pow(yDist, 2));
        if (totalDistance < closestDist) {
          closestDist = totalDistance;
          closestTag = tagId;
        }
      }
    }

    if (closestTag < 1 || closestTag > 22) {
      overrideSwerve = false;
      return;
    }

    final Pose3d desiredTag = this.aprilTagFieldLayout.getTagPose(closestTag).get();
    final Transform3d robotVector = desiredTag.minus(estimatedRobotPose.estimatedPose);

    overrideSwerve = true;

    moveX = (-xPID.calculate(-robotVector.getX()) * MaxSpeed) / 4;
    moveX = xPID.atSetpoint() ? 0 : moveX;
    moveY = (-yPID.calculate(-robotVector.getY()) * MaxSpeed) / 4;
    moveY = yPID.atSetpoint() ? 0 : moveY;
    rotation = desiredTag.getRotation().toRotation2d();
  }
}
