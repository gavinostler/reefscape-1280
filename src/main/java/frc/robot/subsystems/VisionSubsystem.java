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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

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
      .withDeadband(MaxSpeed * 0.01)
      .withRotationalDeadband(MaxAngularRate * 0.01) // Add a 10% deadband
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
  private Pose2d estimatedRobotPose;
  private Optional<PhotonPipelineResult> latestPipelineResult;
  public Rotation2d operatorForward = new Rotation2d();
  public Pose2d drivetrainRobotPose;
  /* estimated time the frame was taken, in the Time Sync Server's time base (nt::Now). */
  public double lastPoseUpdate = 0.0;  
  private static int[] reefIds = new int[] {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};

  public VisionSubsystem() {
    robotToFrontCam =
        new Transform3d(
            new Translation3d(0, 0.12, 0.5),
            new Rotation3d(0, Math.toRadians(0), Math.toRadians(0)));
    photonPoseEstimator =
        new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, robotToFrontCam);

  }

  @Override
  public void periodic() {
    if (!frontCamera.isConnected()) {estimatedRobotPose = null; return;}
    
    final List<PhotonPipelineResult> pipelineResults = frontCamera.getAllUnreadResults();

    if (pipelineResults.size() > 0) {
      latestPipelineResult = Optional.of(pipelineResults.get(0)); // optional because index can be null if photonvision disconnects
       // depends on latest pipeline result
       if (latestPipelineResult.isPresent()) {updateEstimatedRobotPose(latestPipelineResult.get());} else {
        estimatedRobotPose = null;
       };
    } else {
      estimatedRobotPose = null;
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
        .withHeadingPID(1.2, 0, 0);
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
    this.estimatedRobotPose = possiblePose.get().estimatedPose.toPose2d().plus(new Transform2d(0,0, new Rotation2d(Math.toRadians(DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 180 : 0))));
    this.lastPoseUpdate = pipeline.getTimestampSeconds();
  }

  /*
   * Get the estimated vision robot pose as 2d
   */
  public Pose2d getEstimatedPose2d() {
    return estimatedRobotPose;
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
    if (drivetrainRobotPose == null) {
      warning = "NO TARGETS/INVALID ROBOT POSE";
      overrideSwerve = false;
      return;
    }
    // no pose, obviously

    int closestTag = -1;
    double closestDist = Double.MAX_VALUE;
      for (int tagId : reefIds) {

        Pose3d tagPose = this.aprilTagFieldLayout.getTagPose(tagId).get();

        Pose2d dist = tagPose.toPose2d().relativeTo(drivetrainRobotPose);
        double xDist = dist.getMeasureX().magnitude();
        double yDist = dist.getMeasureY().magnitude();

        double totalDistance = Math.sqrt(Math.pow(xDist, 2) + Math.pow(yDist, 2));
        if (totalDistance < closestDist) {
          closestDist = totalDistance;
          closestTag = tagId;
        }
      }

    if (closestTag < 1 || closestTag > 22) {
      warning = "DISABLED BY INVALID ID";
      overrideSwerve = false;
      return;
    }

    final Pose3d desiredTag = this.aprilTagFieldLayout.getTagPose(closestTag).get();
    
    final Pose2d desiredTag2d = desiredTag.toPose2d().plus(new Transform2d(1.1, 0.0, new Rotation2d()));
    final Pose2d robotVector2d = drivetrainRobotPose;
    final Pose2d adjustedPosition = desiredTag2d.relativeTo(robotVector2d);

    final double pidCalc = Math.min(Math.max(pid.calculate(adjustedPosition.getTranslation().getNorm(), 0), -1),1);
    final Pose2d vector = adjustedPosition.times(pidCalc);
    
    overrideSwerve = true;
    lastRobotVector = vector;
    
    moveX = Math.min(Math.max(-(vector.getMeasureX().magnitude() * 5), -MaxSpeed/5), MaxSpeed/5);
    moveY = Math.min(Math.max(-(vector.getMeasureY().magnitude() * 5), -MaxSpeed/5), MaxSpeed/5);
    rotation = desiredTag2d.getRotation();
  }
  
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Vision");
    
    builder.addStringProperty("Last Warning", () -> warning, null);
    
    builder.addDoubleProperty("Robot Vector X Magnitude", () -> lastRobotVector.getMeasureX().magnitude(), null);
    builder.addDoubleProperty("Robot Vector Y Magnitude", () -> lastRobotVector.getMeasureY().magnitude(), null);
    builder.addStringProperty("Robot Vector Rotation", () -> lastRobotVector.getRotation().toString(), null);
    builder.addStringProperty("Robot Pose (Vision)", () -> {if (estimatedRobotPose == null) return ""; return estimatedRobotPose.toString();}, null);
    builder.addStringProperty("Robot Pose (Drivetrain + Vision)", () -> {if (drivetrainRobotPose == null) return ""; return drivetrainRobotPose.toString();}, null);
    
    builder.addStringProperty("Desired Rotation", () -> rotation.toString(), null);
    builder.addDoubleProperty("Move X", () -> moveX, null);
    builder.addDoubleProperty("Move Y", () -> moveY, null);
  }
}


