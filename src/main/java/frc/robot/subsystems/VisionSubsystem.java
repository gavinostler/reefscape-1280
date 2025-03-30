package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

import java.lang.StackWalker.Option;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionSubsystem extends SubsystemBase {

  private PIDController Xpid = new PIDController(2, 0.02, 0.2);
  private PIDController Ypid = new PIDController(2, 0.02, 0.2);
  private PIDController steerpid = new PIDController(1, 0, 0);
  private LinearFilter xFilter = LinearFilter.movingAverage(3);
  private LinearFilter yFilter = LinearFilter.movingAverage(3);



  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 3/4 of a rotation per second max
  // angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
    new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.01)
      .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.Velocity);

  private double rotation = 0.0;
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
  public Pose3d lastVisionPose;
  public Pose2d desPos;
  /* estimated time the frame was taken, in the Time Sync Server's time base (nt::Now). */
  public double lastPoseUpdate = 0.0;  
  private static int[] reefIds = new int[] {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
  private Supplier<Pose2d> drivetrain;
  private SequentialCommandGroup cc = new SequentialCommandGroup();


  public VisionSubsystem() {
    robotToFrontCam =
        new Transform3d(
            new Translation3d(0, 0.14, 0.5),
            new Rotation3d(0, Math.toRadians(0), Math.toRadians(0)));
    photonPoseEstimator =
        new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToFrontCam);
    Xpid.setTolerance(0.01);
    Ypid.setTolerance(0.01);
    steerpid.setTolerance(Math.toRadians(0.5));
  }

  public void setDrivetrain( Supplier<Pose2d> dr) {
    drivetrain = dr;
  }

  @Override
  public void periodic() {
    if (!frontCamera.isConnected()) {estimatedRobotPose = null; return;}
    
    final List<PhotonPipelineResult> pipelineResults = frontCamera.getAllUnreadResults();

    if (pipelineResults.size() > 0) {
      latestPipelineResult = Optional.of(pipelineResults.get(0)); // optional because index can be null if photonvision disconnects
       // depends on latest pipeline result
       if (latestPipelineResult.isPresent() && drivetrainRobotPose != null) {updateEstimatedRobotPose(latestPipelineResult.get());} else {
        estimatedRobotPose = null;
       };
    } else {
      estimatedRobotPose = null;
    }
  }

  /*
   * The override, returning a FieldCentric command
   */
  public FieldCentric drive() {
    FieldCentric dr = drive
    .withVelocityX(moveX) // For now, do not allow movement
    .withVelocityY(moveY) // For now, do not allow movement
    .withRotationalRate(this.rotation);

    return dr;
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
    cc.cancel();
    overrideSwerve = false;
  }

  /*
   * Updates the estimated robot pose for vision.
   */
  private void updateEstimatedRobotPose(PhotonPipelineResult pipeline) {
    if (lastVisionPose != null) photonPoseEstimator.setReferencePose(lastVisionPose);
    final Optional<EstimatedRobotPose> possiblePose = photonPoseEstimator.update(pipeline);

    if (possiblePose.isEmpty()) {this.estimatedRobotPose = null; return;}
    final Pose2d possible2dPose = possiblePose.get().estimatedPose.toPose2d();
    this.estimatedRobotPose = new Pose2d(possible2dPose.getX(), possible2dPose.getY(), drivetrainRobotPose.getRotation());
    this.lastVisionPose = possiblePose.get().estimatedPose;
    this.lastPoseUpdate = possiblePose.get().timestampSeconds;
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
  public Optional<Pose2d> reefAlign() {
    if (latestPipelineResult.isEmpty()) {
          warning = "NO LATEST PIPELINE";
          overrideSwerve = false;
          return Optional.of(null);
        }
    
        // final PhotonTrackedTarget bestTarget = pipeline.get().getBestTarget();
        if (drivetrainRobotPose == null) {
          warning = "NO TARGETS/INVALID ROBOT POSE";
          overrideSwerve = false;
          return Optional.of(null);
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
          return Optional.of(null);
        }
    
        final Pose3d desiredTag = this.aprilTagFieldLayout.getTagPose(closestTag).get();
        
        final Pose2d desiredTag2d = desiredTag.toPose2d().plus(new Transform2d(0.1, 0.0, new Rotation2d()));
        overrideSwerve = true;

        Command p = AutoBuilder.pathfindToPose(desiredTag2d, new PathConstraints(1.5, 2, Units.degreesToRadians(3000), Units.degreesToRadians(2000)), 0.0).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
        Command in = AutoBuilder.pathfindToPose(desiredTag2d.plus(new Transform2d(0.6, 0.0, new Rotation2d())), new PathConstraints(1, 2, Units.degreesToRadians(3000), Units.degreesToRadians(2000)), 0.0).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

        cc.cancel();
        cc =  new SequentialCommandGroup(in, p);
        cc.schedule();
  
        return Optional.of(desiredTag2d);
      }
    
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Vision");
    
    builder.addStringProperty("Last Warning", () -> warning, null);
    
    builder.addDoubleProperty("Robot Vector Magnitude Rot", () -> rotation, null);
    builder.addStringProperty("Robot Pose (Vision)", () -> {if (estimatedRobotPose == null) return ""; return estimatedRobotPose.toString();}, null);
    // builder.addStringProperty("Robot Pose (Drivetrain + Vision)", () ->{return drivetrain == null || drivetrain.get() == null ? "" : drivetrain.get().toString();}, null);
    builder.addStringProperty("Des Robot Pose (Vision)", () -> {if (desPos == null) return ""; return desPos.toString();}, null);

    builder.addDoubleProperty("Desired Rotation", () -> rotation, null);
    builder.addDoubleProperty("Move X", () -> moveX, null);
    builder.addDoubleProperty("Move Y", () -> moveY, null);
    builder.addDoubleProperty("Move Y", () -> rotation, null);

    builder.addDoubleProperty("PID KP", () -> Xpid.getP(), (double val) -> {
      Xpid.setP(val);
      Ypid.setP(val);
    });
    builder.addDoubleProperty("PID KI", () -> Xpid.getI(), (double val) -> {
      Xpid.setI(val);
      Ypid.setI(val);
    });
    builder.addDoubleProperty("PID KD", () -> Xpid.getD(), (double val) -> {
      Xpid.setD(val);
      Ypid.setD(val);
    });

    builder.addDoubleProperty("R PID KP", () ->  steerpid.getP(), (double val) -> {
      steerpid.setP(val);
    });
    builder.addDoubleProperty("R PID KI", () -> steerpid.getI(), (double val) -> {
      steerpid.setI(val);
    });
    builder.addDoubleProperty("R  PID KD", () -> steerpid.getD(), (double val) -> {
      steerpid.setD(val);
    });
  }
}


