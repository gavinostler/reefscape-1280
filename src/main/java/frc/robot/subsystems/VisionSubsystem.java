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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.sendable.SendableBuilder;
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

  private PIDController pid = new PIDController(1.2, 0, 0);

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
  private String warning = "";
  private Pose2d lastRobotVector = new Pose2d();

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
            new Rotation3d(0, Math.toRadians(30), Math.toRadians(0)));
    photonPoseEstimator =
        new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, robotToFrontCam);

    pid.setTolerance(0.0001);
  }

  @Override
  public void periodic() {
    if (!frontCamera.isConnected()) return;
    
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
        .withHeadingPID(10, 0, 0);
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
      warning = "NO LATEST PIPELINE";
      overrideSwerve = false;
      return;
    }

    // final PhotonTrackedTarget bestTarget = pipeline.get().getBestTarget();
    final List<PhotonTrackedTarget> targets = latestPipelineResult.get().getTargets();
    if (estimatedRobotPose == null || targets.isEmpty()) {
      warning = "NO TARGETS/INVALID ROBOT POSE";
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

        Pose3d dist = tagPose.relativeTo(estimatedRobotPose.estimatedPose);
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
      warning = "DISABLED BY INVALID ID";
      overrideSwerve = false;
      return;
    }

    final Pose3d desiredTag = this.aprilTagFieldLayout.getTagPose(closestTag).get();
    
    final Pose2d desiredTag2d = desiredTag.toPose2d().transformBy(new Transform2d(1.0, 0.0, new Rotation2d(0)));
    final Pose2d robotVector2d = estimatedRobotPose.estimatedPose.toPose2d();
    final Pose2d adjustedPosition = desiredTag2d.relativeTo(robotVector2d);

    final double pidCalc = pid.calculate(adjustedPosition.getTranslation().getNorm(), 0);
    final Pose2d vector = adjustedPosition.times(pidCalc);
    
    overrideSwerve = true;
    lastRobotVector = vector;
    
    moveX = pid.atSetpoint() ? 0 : -(vector.getX() * MaxSpeed/2);
    moveY = pid.atSetpoint() ? 0 : -(vector.getY() * MaxSpeed/2);
    rotation = desiredTag2d.getRotation();
  }
  
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Vision");
    
    builder.addStringProperty("Last Warning", () -> warning, null);
    
    builder.addDoubleProperty("Robot Vector X Magnitude", () -> lastRobotVector.getMeasureX().magnitude(), null);
    builder.addDoubleProperty("Robot Vector Y Magnitude", () -> lastRobotVector.getMeasureY().magnitude(), null);
    builder.addStringProperty("Robot Vector Rotation", () -> lastRobotVector.getRotation().toString(), null);
    builder.addStringProperty("Robot Pose (Unadjusted)", () -> estimatedRobotPose.estimatedPose.toPose2d().toString(), null);
    
    builder.addStringProperty("Desired Rotation", () -> rotation.toString(), null);
    builder.addStringProperty("Desired Rotation", () -> rotation.toString(), null);
  }
}


