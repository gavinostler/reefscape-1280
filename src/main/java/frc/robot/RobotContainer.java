// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Driver;
import frc.robot.Constants.GroundIntake;
// import frc.robot.aesthetic.Colors;
import frc.robot.controller.AgnosticController;
import frc.robot.generated.TunerConstants;
import frc.robot.state.State;
import frc.robot.state.Validator;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@SuppressWarnings("unused") // Blah blah
public class RobotContainer {
  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double LowMaxSpeed = MaxSpeed / 2; // TODO: set lowered speed for precise alignment
  private boolean loweredSpeed = false;

  private double MaxAngularRate =
      RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 3/4 of a rotation per second max
  private double LowMaxAngularRate =
      MaxAngularRate / 2; // TODO: set lowered rate for precise alignment
  // angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.02)
          .withRotationalDeadband(MaxAngularRate * 0.02) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.Velocity); // Use open-loop control for drive motors

  // The robot's subsystems and commands are defined here...
  public final CommandSwerveDrivetrain drivetrain; // declared later due to NamedCommands
  private final Validator validator = new Validator();
  public final ElevatorSubsystem elevator = new ElevatorSubsystem(validator);
  private final ShooterSubsystem shooter = new ShooterSubsystem(validator);
  private final GroundIntakeSubsystem groundIntake = new GroundIntakeSubsystem(validator);
  private final AgnosticController driverController =
      new AgnosticController(Driver.kDriverControllerPort);
  public final AgnosticController operatorController =
      driverController; // Aliased to main controller for now, TODO
  private final VisionSubsystem vision = new VisionSubsystem();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  // private final Colors colors = new Colors();

  // sendable for choosing autos
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // NOTE: robot must be in stowed state when initializing because of assumptions
    validator.elevator = elevator;
    validator.shooter = shooter;
    validator.groundIntake = groundIntake;
    // Configure the trigger bindings
    // colors.animateCandle(Colors.Effect.CHROMA);
    registerNamedCommands();
    drivetrain = TunerConstants.createDrivetrain(); // AFTER NamedCommands are registered
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser("ventura_auto");
    SmartDashboard.putData("Auto Mode", autoChooser);
    SmartDashboard.putData("elevator", elevator);
    SmartDashboard.putData("shooter", shooter);
    SmartDashboard.putData("ground intake", groundIntake);
    SmartDashboard.putData("validator", validator);

    elevator.setState(elevator.getState());
    shooter.setState(shooter.getState());
    groundIntake.setState(groundIntake.getState());
  }

  /** Register named commands, for use in autonomous */
  public void registerNamedCommands() {
    // Ground intake commands
    NamedCommands.registerCommand(
        "groundIntakeDown",
        groundIntake.runOnce(() -> groundIntake.setState(State.GroundIntake.DOWN)));
    NamedCommands.registerCommand(
        "groundIntakeUp", groundIntake.runOnce(() -> groundIntake.setState(State.GroundIntake.UP)));
    NamedCommands.registerCommand(
        "groundIntakeOff", groundIntake.runOnce(() -> groundIntake.off()));

    // All shooter commands
    NamedCommands.registerCommand(
        "enableShooterOut", shooter.runOnce(() -> shooter.enableShooter(false)));
    NamedCommands.registerCommand(
        "enableShooterIn", shooter.runOnce(() -> shooter.enableShooter(true)));
    NamedCommands.registerCommand(
        "disableShooter", shooter.runOnce(() -> shooter.disableShooter()));
    NamedCommands.registerCommand(
        "enableFeedOut", shooter.runOnce(() -> shooter.enableFeed(false)));
    NamedCommands.registerCommand("enableFeedIn", shooter.runOnce(() -> shooter.enableFeed(true)));
    NamedCommands.registerCommand("disableFeed", shooter.runOnce(() -> shooter.disableFeed()));
    NamedCommands.registerCommand("runIntakeAlgae", shooter.runOnce(shooter::intakeAlgae));
    NamedCommands.registerCommand("runShootAlgae", shooter.runShootAlgae());
    NamedCommands.registerCommand("runProcessor", runProcessor());

    // Elevator commands
    NamedCommands.registerCommand(
        "moveElevatorGroundIntake",
        elevator.runOnce(() -> elevator.setState(State.Elevator.GROUND_INTAKE)));
    NamedCommands.registerCommand(
        "moveElevatorL1", elevator.runOnce(() -> elevator.setState(State.Elevator.L1)));
    NamedCommands.registerCommand(
        "moveElevatorL2", elevator.runOnce(() -> elevator.setState(State.Elevator.L2)));
    NamedCommands.registerCommand(
        "moveElevatorShoot", elevator.runOnce(() -> elevator.setState(State.Elevator.SHOOT)));

    NamedCommands.registerCommand("L1", runL1());
    NamedCommands.registerCommand("L2", runL2());

    // Stow after auto
    NamedCommands.registerCommand("runStowSubsystems", runStowSubsystems());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via agnostic controllers for {@link AgnosticController Xbox/PS4}
   */
  public void configureBindings() {
    if (Constants.kEnableSysId) {
      bindSysId();
    }

    // Driver Controls
    // TODO: way to do precise movement near reef
    int moveTaps = 10;
    int rotationTaps = 10;
    LinearFilter filterX = LinearFilter.movingAverage(moveTaps);
    LinearFilter filterY = LinearFilter.movingAverage(moveTaps);
    LinearFilter filterRotation = LinearFilter.movingAverage(rotationTaps);
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () -> {
              if (vision.isOverridingSwerve()) {
                return vision.drive();
              }
              double speed;
              double angularRate;
              double fraction = getSwerveSpeedFraction();
              if (loweredSpeed) {
                speed = LowMaxSpeed;
                angularRate = LowMaxAngularRate;
              } else {
                speed = MaxSpeed * fraction;
                angularRate = MaxAngularRate * fraction;
              }
              return drive
                  .withVelocityX(
                      filterY.calculate(MathUtil.applyDeadband(driverController.getLeftY(), 0.1))
                          * speed)
                  .withVelocityY(
                      filterX.calculate(MathUtil.applyDeadband(driverController.getLeftX(), 0.1))
                          * speed)
                  .withRotationalRate(
                      filterRotation.calculate(
                              -MathUtil.applyDeadband(driverController.getRightX(), 0.1))
                          * angularRate);
            }));

    driverController
        .resetHeading()
        .onTrue(drivetrain.runOnce(() -> drivetrain.resetRotation(Rotation2d.kZero)));

    drivetrain.registerTelemetry(logger::telemeterize);

    // Operator Controls
    // TODO: delay transitions so that state machine always reflects reality
    // Elevator controls
    operatorController.b().onTrue(runL1());
    operatorController.y().onTrue(runL2());
    operatorController.leftBumper().onTrue(runBarge());
    operatorController.rightBumper().onTrue(vision.run(vision::reefAlign)).onFalse(vision.runOnce(vision::stop));
    operatorController.povUp().onTrue(elevator.runOnce(() -> elevator.moveState(false)));
    operatorController.povDown().onTrue(elevator.runOnce(() -> elevator.moveState(true)));

    // GroundIntake controls
    operatorController.a().onTrue(runGroundIntake());
    // operatorController.a().onTrue(groundIntake.runOnce(groundIntake::toggleState));

    // Shooter arm controls
    // Note: will not do anything if an uninterruptible shoot procedure is running
    operatorController.povRight().onTrue(shooter.runOnce(() -> shooter.moveState(false)));
    operatorController.povLeft().onTrue(shooter.runOnce(() -> shooter.moveState(true)));

    // Shooter motor controls
    operatorController
        .leftTrigger()
        .onTrue(
            shooter.runOnce(
                () -> {
                  shooter.intakeAlgae();
                  groundIntake.enablePulley();
                }))
        .onFalse(
            shooter.runOnce(
                () -> {
                  shooter.disableShooter();
                  shooter.brakeFeed();
                  groundIntake.disablePulley();
                }));
    operatorController.rightTrigger().onTrue(shooter.runShootAlgae());
    operatorController.x().onTrue(runProcessor());

    // Reset heading
    operatorController
        .resetHeading()
        .onTrue(
            drivetrain.runOnce(
                () -> {
                  Rotation2d x = drivetrain.getState().Pose.getRotation();
                  x =
                      x.rotateBy(new Rotation2d(Math.PI))
                          .rotateBy(drivetrain.getOperatorForwardDirection());
                  drivetrain.resetRotation(x);
                }));

    // Stow all subsystems
    operatorController.rightStick().onTrue(runStowSubsystems());
  }

  void bindSysId() {
    driverController
        .back()
        .and(driverController.y())
        .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driverController
        .back()
        .and(driverController.x())
        .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driverController
        .start()
        .and(driverController.y())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driverController
        .start()
        .and(driverController.x())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    driverController
        .start()
        .and(driverController.povDown())
        .onTrue(drivetrain.runOnce(drivetrain::sysIdCycleRoutine));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Command runGroundIntake() {
    return new SequentialCommandGroup(
            new InstantCommand(() -> setSafety(false)), // we don't need safety for this
            groundIntake.runOnce(
                () -> groundIntake.setState(State.GroundIntake.DOWN)), // move ground intake down
            new WaitUntilCommand(groundIntake::atSetpoint).withTimeout(1.0), // wait until it's set
            elevator.runOnce(
                () ->
                    elevator.setState(
                        State.Elevator.GROUND_INTAKE)), // move elevator to ground intake position
            new WaitUntilCommand(elevator::atSetpoint).withTimeout(2.0), // wait until it's set
            shooter.runOnce(
                () ->
                    shooter.setState(
                        State.Shooter.GROUND_INTAKE)), // move shooter to ground intake angle
            new WaitUntilCommand(shooter::atSetpoint).withTimeout(1.0), // wait until it's set
            new InstantCommand(() -> setSafety(true)) // put safety back
            )
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming); // don't get interrupted
  }

  public Command runL1() {
    return new SequentialCommandGroup(
            new InstantCommand(() -> setSafety(false)), // we don't need safety for this
            elevator.runOnce(
                () -> elevator.setState(State.Elevator.L1)), // move elevator to L1 position
            new WaitUntilCommand(elevator::atSetpoint).withTimeout(2.0), // wait until it's set
            shooter.runOnce(
                () ->
                    shooter.setState(State.Shooter.REEF_INTAKE)), // move shooter to L1 intake angle
            new WaitUntilCommand(shooter::atSetpoint).withTimeout(2.0), // wait until it's set
            groundIntake.runOnce(
                () ->
                    groundIntake.setState(
                        State.GroundIntake.UP)), // move ground intake up (away from reef)
            groundIntake.runKickUp(0.4),
            new WaitUntilCommand(groundIntake::atSetpoint).withTimeout(1.0), // wait until it's set
            new InstantCommand(() -> setSafety(true)) // put safety back
            )
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming); // don't get interrupted
  }

  public Command runL2() {
    return new SequentialCommandGroup(
            new InstantCommand(() -> setSafety(false)), // we don't need safety for this
            elevator.runOnce(
                () -> elevator.setState(State.Elevator.L2)), // move elevator to L1 position
            new WaitUntilCommand(elevator::atSetpoint).withTimeout(2.0), // wait until it's set
            shooter.runOnce(
                () ->
                    shooter.setState(State.Shooter.REEF_INTAKE)), // move shooter to L1 intake angle
            new WaitUntilCommand(shooter::atSetpoint).withTimeout(2.0), // wait until it's set
            groundIntake.runOnce(
                () ->
                    groundIntake.setState(
                        State.GroundIntake.UP)), // move ground intake up (away from reef)
            groundIntake.runKickUp(0.4),
            new WaitUntilCommand(groundIntake::atSetpoint).withTimeout(1.0), // wait until it's set
            new InstantCommand(() -> setSafety(true)) // put safety back
            )
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming); // don't get interrupted
  }

  public Command runBarge() {
    return new SequentialCommandGroup(
            new InstantCommand(() -> setSafety(false)),
            elevator.runOnce(() -> elevator.setState(State.Elevator.SHOOT)),
            new WaitUntilCommand(elevator::atSetpoint).withTimeout(1.0),
            shooter.runOnce(() -> shooter.setState(State.Shooter.SHOOT)),
            new WaitUntilCommand(shooter::atSetpoint).withTimeout(1.0),
            new InstantCommand(() -> setSafety(true)))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public Command runProcessor() {
    return new SequentialCommandGroup(
            new InstantCommand(() -> setSafety(false)),
            groundIntake.runOnce(() -> GroundIntake.intakePID.setSetpoint(0.006)),
            new WaitUntilCommand(groundIntake::atSetpoint).withTimeout(1.0),
            shooter.runOnce(() -> shooter.moveArmAngle(0.0)),
            new WaitUntilCommand(shooter::atSetpoint).withTimeout(2.0),
            elevator.runOnce(() -> elevator.moveHeight(0.266)),
            new WaitUntilCommand(elevator::atSetpoint).withTimeout(2.0),
            new InstantCommand(() -> setSafety(true))
            // NOTE: state is out of sync but its quite safe so this can be ignored
            )
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public Command runStowSubsystems() {
    return new SequentialCommandGroup(
            new InstantCommand(() -> setSafety(false)),
            groundIntake.runOnce(
                () ->
                    groundIntake.setState(
                        State.GroundIntake.DOWN)), // move ground intake out of the way
            new WaitUntilCommand(groundIntake::atSetpoint).withTimeout(1.0), // wait until it's set
            shooter.runOnce(() -> shooter.setState(State.Shooter.STOW)), // stow shooter
            new WaitUntilCommand(shooter::atSetpoint).withTimeout(3.0), // wait until it's set
            elevator.runOnce(() -> elevator.setState(State.Elevator.BOTTOM)), // stow elevator
            new WaitUntilCommand(elevator::atSetpoint).withTimeout(3.0), // wait until it's set
            groundIntake.runOnce(() -> groundIntake.setState(State.GroundIntake.UP)),
            groundIntake.runKickUp(0.4),
            new WaitUntilCommand(groundIntake::atSetpoint).withTimeout(1.0), // wait until it's set
            new InstantCommand(() -> setSafety(true)))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  // Origin from Blue Alliance bottom-right corner
  class Field {
    AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    /**
     * Aligns to the vertical corresponding to the relevant net and side.
     *
     * @param alliance Our alliance
     * @param distance Desired horizontal (X) distance from barge tag in meters
     */
    Command alignBarge(Alliance alliance, double distance) {
      // If we are Blue use the top net (id 14, 4) else use bottom net (id 15, 5)
      double leftSideTagX =
          (alliance == Alliance.Blue ? tagLayout.getTagPose(14) : tagLayout.getTagPose(15))
              .get()
              .getX();
      double rightSideTagX =
          (alliance == Alliance.Blue ? tagLayout.getTagPose(4) : tagLayout.getTagPose(5))
              .get()
              .getX();

      return drivetrain.getAlignToFieldPosition(
          () -> {
            var driveTranslation = drivetrain.getState().Pose.getTranslation();
            if (Math.abs(driveTranslation.getX() - leftSideTagX)
                < Math.abs(driveTranslation.getX() - rightSideTagX)) {
              // Left side is closer
              return new Pose2d(
                  leftSideTagX - distance, driveTranslation.getY(), new Rotation2d(Math.PI));
            } else {
              // Right side is closer
              return new Pose2d(
                  rightSideTagX + distance, driveTranslation.getY(), new Rotation2d(0.0));
            }
          });
    }
  }

  public void setSafety(boolean state) {
    Validator.safetyEnabled = state;
  }

  private double getSwerveSpeedFraction() {
    // https://www.desmos.com/calculator/ghdph63sxf
    final double slowingHeight = 0.2;
    final double minSpeedFraction = 0.30;
    final double curveCoefficient = (minSpeedFraction - 1) / Math.pow(slowingHeight - 1, 2);
    final double height =
        MathUtil.clamp(
            elevator.getHeight() + 0.1 * Math.sin(shooter.getArmAngle() * 2 * Math.PI), 0, 1);
    double speedFraction;
    if (height < slowingHeight) {
      speedFraction = 1.0;
    } else {
      double curveTerm = curveCoefficient * (height - slowingHeight) * (height - 1);
      speedFraction = (1 - minSpeedFraction) / (slowingHeight - 1) * (height - slowingHeight) + 1;
      speedFraction += curveTerm;
    }
    speedFraction = MathUtil.clamp(speedFraction, minSpeedFraction, 1.0);
    return speedFraction;
  }

  /**
   * Incorporate vision estimates into drivetrain pose estimates
   * Use only while vision is running
   * Intended to enhance autonomous
   */
  private void addVisionMeasurement() {
    Pose2d visionEstimate = vision.getEstimatedPose2d();
    if (visionEstimate == null) return;
    Pose2d currentEstimate = drivetrain.getState().Pose;
    double estimatesDistance = currentEstimate.getTranslation().getDistance(visionEstimate.getTranslation());
    // Filter out bad vision measurements
    if (estimatesDistance > 1.0) return;
    drivetrain.addVisionMeasurement(visionEstimate, Timer.getFPGATimestamp());
  }
}
