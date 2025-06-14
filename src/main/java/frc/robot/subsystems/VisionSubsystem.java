package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;
import java.util.function.Supplier;


public class VisionSubsystem extends SubsystemBase {

  private String warning = "";

  private AprilTagFieldLayout aprilTagFieldLayout =
    AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  Camera[] cameras = {
    new Camera("front", aprilTagFieldLayout, new Transform3d(
      new Translation3d(Units.inchesToMeters(13.5), Units.inchesToMeters(5.5), Units.inchesToMeters(8)),
      new Rotation3d(0, Math.toRadians(20), Math.toRadians(0))
    )),
    new Camera("back", aprilTagFieldLayout, new Transform3d(
      new Translation3d(-Units.inchesToMeters(13.5), -Units.inchesToMeters(5.5), 0.5),
      new Rotation3d(0, Math.toRadians(20), Math.toRadians(180))
    ))
  };
  private Pose2d estimatedRobotPose;
  public double lastPoseUpdate = 0.0; // estimated time the frame was taken, in the Time Sync Server's time base (nt::Now). Currently unused, but should be used lmao
  private SendableChooser<Mode> chooser = new SendableChooser<>();
  private Mode currentMode = Mode.REEF;

  private Supplier<Pose2d> drivetrain;
  private Field2d fieldview = new Field2d();

  public enum Mode {
    REEF,
    PROCESSOR,
    BARGE,
  }

  public VisionSubsystem() {
  }

  public void setDrivetrain(Supplier<Pose2d> dr) {
    drivetrain = dr;
  }

  public Mode getNextMode(boolean reverse) {
    return reverse ? switch (getMode()) {
      case REEF -> Mode.REEF;
      case PROCESSOR -> Mode.REEF;
      case BARGE -> Mode.PROCESSOR;
    } : switch (getMode()) {
      case REEF -> Mode.PROCESSOR;
      case PROCESSOR -> Mode.BARGE;
      case BARGE -> Mode.BARGE;
    };
  }

  public void cycleMode(boolean reverse) {
    setMode(getNextMode(reverse));
  }

  public void setMode(Mode mode) {
    currentMode = mode;
  }

  @Override
  public void periodic() {
    Pose2d newEstimatedPose = null;
    double newLatencyTime = this.lastPoseUpdate;
    for (Camera camera : cameras) {
      Optional<Pose2d> estPose = camera.getEstimatedRobotPose(drivetrain.get());
      if (estPose.isEmpty()) {
        continue;
      }
      newEstimatedPose = estPose.get();
      newLatencyTime = camera.getLatencyTime();
      fieldview.setRobotPose(newEstimatedPose);
    }

    this.estimatedRobotPose = newEstimatedPose;
    this.lastPoseUpdate = newLatencyTime;
  }


  /*
   * Get the estimated vision robot pose as 2d
   */
  public Pose2d getEstimatedPose2d() {
    return estimatedRobotPose;
  }

  public Optional<Integer> getClosestTagId(int[] tagList) {
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
    return currentMode;
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

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Vision");

    chooser.setDefaultOption("Reef", Mode.REEF);
    chooser.addOption("Barge", Mode.BARGE);
    chooser.addOption("Processor", Mode.PROCESSOR);
    chooser.onChange((Mode newMode) -> {
      setMode(newMode);
    });

    SmartDashboard.putData("Vision Mode Chooser", chooser);
    SmartDashboard.putData("Vision Field", fieldview);

    builder.addStringProperty("Current Vision Mode", () -> getMode().toString(), null);

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
