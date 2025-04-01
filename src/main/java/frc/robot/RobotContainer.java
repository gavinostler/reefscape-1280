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
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Driver;
import frc.robot.Constants.GroundIntake;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.Vision;
// import frc.robot.aesthetic.Colors;
import frc.robot.controller.AgnosticController;
import frc.robot.controller.XboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.state.State;
import frc.robot.state.State.Elevator;
import frc.robot.state.Validator;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.Mode;

import java.time.InstantSource;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@SuppressWarnings("unused") // Blah blah
public class RobotContainer {

  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.6; // kSpeedAt12Volts desired top speed
  private double LowMaxSpeed = MaxSpeed / 2; // TODO: set lowered speed for precise alignment
  private boolean loweredSpeed = false;
  
  private Command lastVisionCommand;

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
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(); // declared later due to NamedCommands
  private final Validator validator = new Validator();
  public final ElevatorSubsystem elevator = new ElevatorSubsystem(validator);
  private final ShooterSubsystem shooter = new ShooterSubsystem(validator);
  private final GroundIntakeSubsystem groundIntake = new GroundIntakeSubsystem(validator);
  private final AgnosticController dummy =  new AgnosticController(Driver.kDriverControllerPort);
  private final XboxController driverController =
      new XboxController(Driver.kDriverControllerPort);
  public final XboxController operatorController =
      driverController; // Aliased to main controller for now, TODO
  public final VisionSubsystem vision = new VisionSubsystem();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  // private final Colors colors = new Colors();

  // sendable for choosing autos
  private SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // NOTE: robot must be in stowed state when initializing because of assumptions
    // Configure the trigger bindings
    // colors.animateCandle(Colors.Effect.CHROMA);
    registerNamedCommands();
    autoChooser = AutoBuilder.buildAutoChooser("ventura_auto");
    SmartDashboard.putData("Auto Mode", autoChooser);
    configureBindings();
  }

  public void updateDashboard() {
    SmartDashboard.putData("elevator", elevator);
    SmartDashboard.putData("shooter", shooter);
    SmartDashboard.putData("ground intake", groundIntake);
    SmartDashboard.putData("validator", validator);
    SmartDashboard.putData("v", vision);
  }

  public void setInitalStates() {
    validator.elevator = elevator;
    validator.shooter = shooter;
    validator.groundIntake = groundIntake;
    elevator.setState(elevator.getState());
    shooter.setState(shooter.getState());
    groundIntake.setState(groundIntake.getState());
  }

  /** Register named commands, for use in autonomous */
  public void registerNamedCommands() {

    // All shooter commands
    NamedCommands.registerCommand("runShootAlgae", shooter.runShootAlgae());
    NamedCommands.registerCommand("runIntake", intakeOn());
    NamedCommands.registerCommand("stopIntake",intakeOff());
        
    // State commands
    NamedCommands.registerCommand(
        "runL1", runL1());
    NamedCommands.registerCommand(
        "runL2", runL2());
    NamedCommands.registerCommand(
        "runBarge", runBarge());

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
    int moveTaps = 20;
    int rotationTaps = 10;
    LinearFilter filterX = LinearFilter.movingAverage(moveTaps);
    LinearFilter filterY = LinearFilter.movingAverage(moveTaps);
    LinearFilter filterRotation = LinearFilter.movingAverage(rotationTaps);
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () -> {
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
                      filterY.calculate(MathUtil.applyDeadband(-driverController.getLeftY(), 0.1))
                          * speed)
                  .withVelocityY(
                      filterX.calculate(MathUtil.applyDeadband(-driverController.getLeftX(), 0.1))
                          * speed)
                  .withRotationalRate(
                      filterRotation.calculate(
                              -MathUtil.applyDeadband(driverController.getRightX(), 0.1))
                          * angularRate);
            }));

    // driverController
    //     .resetHeading()
    //     .onTrue(drivetrain.runOnce(() -> drivetrain.resetRotation(Rotation2d.kZero)));

    drivetrain.registerTelemetry(logger::telemeterize);

    // Operator Controls
    // TODO: delay transitions so that state machine always reflects reality
    // Elevator controls
    operatorController.b().onTrue(runL1());
    operatorController.y().onTrue(runL2());
    operatorController.leftBumper().onTrue(runBarge());
    // operatorController.rightBumper().onTrue(vision.runOnce(() -> vision.reefAlign())).onFalse(vision.runOnce(() -> {vision.removeDefaultCommand(); vision.stop();}));
    operatorController.rightBumper().onTrue(Commands.defer(() -> {
      Command newCommand = switch (vision.getMode()) {
        case BARGE -> closestBargeAlign();
        case REEF -> closestReefAlign();
        case PROCESSOR -> closestProcessorAlign();
      };
      lastVisionCommand = newCommand;
      return newCommand;
    }, new HashSet<>(Arrays.asList(vision)))).onFalse(vision.runOnce(() -> {lastVisionCommand.cancel(); lastVisionCommand = null;}));
    operatorController.povUp().onTrue(elevator.runOnce(() -> elevator.moveState(false)));
    operatorController.povDown().onTrue(elevator.runOnce(() -> elevator.moveState(true)));

    // GroundIntake controls
    operatorController.a().onTrue(runGroundIntake());
    operatorController.back().onTrue(runDislodge());
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
    operatorController.x().onTrue(runProcessor().unless(() -> elevator.getState() == Elevator.BOTTOM));

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
                  Pose2d estPose = vision.getEstimatedPose2d();
                  if (estPose == null) {
                    return;
                  }
                  drivetrain.resetTranslation(estPose.getTranslation());
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
            elevator.runOnce(
                () ->
                    elevator.setState(
                        State.Elevator.GROUND_INTAKE)), // move elevator to ground intake position (safe even if ground intake is still up)
            shooter.runOnce(() -> shooter.setState(State.Shooter.REEF_INTAKE)).unless(() -> shooter.getArmAngle() < 0.1), // if needed, start moving shooter early
            new WaitUntilCommand(elevator::atSetpoint).withTimeout(2.0), // wait until it's set
            shooter.runOnce(
                () ->
                    shooter.setState(
                        State.Shooter.GROUND_INTAKE)), // move shooter to ground intake angle
            new WaitUntilCommand(shooter::atSetpoint).withTimeout(0.3), // don't wait until it's set
            new InstantCommand(() -> setSafety(true)) // put safety back
            )
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming); // don't get interrupted
  }

  public Command runL1() {
    return new SequentialCommandGroup(
            new InstantCommand(() -> setSafety(false)), // we don't need safety for this
            new SequentialCommandGroup(
              shooter.runOnce(() -> shooter.setState(State.Shooter.REEF_INTAKE)), // move arm out of way if needed
              new WaitUntilCommand(shooter::atSetpoint).withTimeout(2.0)
            ).onlyIf(() -> shooter.getArmAngle() < 0.1),
            elevator.runOnce(
              () -> elevator.setState(State.Elevator.L1)), // move elevator to L1 position
            groundIntake.runOnce(
                () ->
                    groundIntake.setState(State.GroundIntake.UP)), // move ground intake up (away from reef)
            new WaitUntilCommand(elevator::atSetpoint).withTimeout(2.0), // wait until it's set
            shooter.runOnce(
                () ->
                    shooter.setState(State.Shooter.REEF_INTAKE)), // move shooter to L1 intake angle
            new WaitUntilCommand(shooter::atSetpoint).withTimeout(1.0), // wait until it's set
            new InstantCommand(() -> setSafety(true)) // put safety back
            )
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming); // don't get interrupted
  }

  public Command runL2() {
    return new SequentialCommandGroup(
            new InstantCommand(() -> setSafety(false)), // we don't need safety for this
            elevator.runOnce(
                () -> elevator.setState(State.Elevator.L2)), // move elevator to L1 position
            groundIntake.runOnce(
                () ->
                    groundIntake.setState(
                        State.GroundIntake.UP)), // move ground intake up (away from reef)
            new WaitUntilCommand(elevator::atSetpoint).withTimeout(2.0), // wait until it's set
            shooter.runOnce(
                () ->
                    shooter.setState(State.Shooter.REEF_INTAKE)), // move shooter to L1 intake angle
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
            new WaitUntilCommand(shooter::atSetpoint).withTimeout(0.3),
            new InstantCommand(() -> setSafety(true)))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public Command runDislodge() {
    return new SequentialCommandGroup(
            new InstantCommand(() -> setSafety(false)),
            groundIntake.runOnce(() -> GroundIntake.intakePID.setSetpoint(0.006)),
            new WaitUntilCommand(groundIntake::atSetpoint).withTimeout(1.0),
            elevator.runOnce(() -> elevator.setState(State.Elevator.GROUND_INTAKE)),
            new WaitUntilCommand(elevator::atSetpoint).withTimeout(2.0),
            shooter.runOnce(() -> shooter.setState(State.Shooter.DOWN)),
            new WaitUntilCommand(shooter::atSetpoint).withTimeout(2.0),
            shooter.runOnce(
                () -> {
                  shooter.intakeAlgae();
                }),
            new WaitCommand(0.7),
            shooter.runOnce(
                    () -> {
                      shooter.disableShooter();
                      shooter.brakeFeed();
                    }),
            shooter.runOnce(() -> shooter.setState(State.Shooter.REEF_INTAKE)),
            new InstantCommand((() -> setSafety(true)))
            )
          .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public Command runProcessor() {
    return new SequentialCommandGroup(
            new InstantCommand(() -> setSafety(false)),
            groundIntake.runOnce(() -> GroundIntake.intakePID.setSetpoint(0.006)),
            new WaitUntilCommand(groundIntake::atSetpoint).withTimeout(1.0),
            shooter.runOnce(() -> shooter.moveArmAngle(0.0)),
            new WaitUntilCommand(shooter::atSetpoint).withTimeout(1.0),
            elevator.runOnce(() -> elevator.moveHeight(0.266)),
            new InstantCommand(() -> setSafety(true))
            // NOTE: state is out of sync but its quite safe so this can be ignored
            )
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }
  
  public Command intakeOn() {
    return shooter.runOnce(
        () -> {
            shooter.intakeAlgae();
            groundIntake.enablePulley();
        });
  }
  
  public Command intakeOff() {
    return shooter.runOnce(
        () -> {
            shooter.disableShooter();
            shooter.brakeFeed();
            groundIntake.disablePulley();
        });
  }

  public Command runIntake() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> setSafety(false)),
        shooter.runOnce(
                () -> {
                  shooter.intakeAlgae();
                  groundIntake.enablePulley();
                }),
        new WaitCommand(1.5),
        shooter.runOnce(
                () -> {
                  shooter.disableShooter();
                  shooter.brakeFeed();
                  groundIntake.disablePulley();
                }),
         new InstantCommand(() -> setSafety(true)))
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
            new WaitUntilCommand(groundIntake::atSetpoint).withTimeout(1.0), // wait until it's set`
            new InstantCommand(() -> setSafety(true)))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public Command closestReefAlign() {
    // patented D.N.K.R G.A.V.I.N (Do Not Kill Robot - General Autonomous Vision Information Networking)
    Optional<Integer> closestTagOptional = vision.getClosestTagId(Vision.reefIds);
    if (closestTagOptional.isEmpty()) return new Command() {};

    int closestTag = closestTagOptional.get();
    final Pose2d desiredTag2d = vision.getTagPose2d(closestTag);
    final boolean isL2 = vision.getIsL2FromTag(closestTag);
    

    Command align = AutoBuilder.pathfindToPose(
      desiredTag2d.plus(Vision.reefAlign),
      new PathConstraints(
        Vision.reefMaxVelocity,
        Vision.reefMaxAcceleration,
        Vision.reefMaxRotationalRate,
        Vision.reefMaxAccelerationRotationalRate
      ),
      0.0
    );


    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        AutoBuilder.pathfindToPose(
      desiredTag2d.transformBy(Vision.reefAlignFar),
      new PathConstraints(
        Vision.reefInMaxVelocity,
        Vision.reefMaxAcceleration,
        Vision.reefMaxRotationalRate,
        Vision.reefMaxAccelerationRotationalRate
      ),
      0.0
    ), // align to tag, but far away so shooter doesnt hit
        isL2 ? runL2() : runL1() // move to L1/L2 based on tag ID
      ), 
      intakeOn(), // self explainatory
      align, // go in so shooter intakes ball
      new ParallelCommandGroup( // parallel to not rip ball apart
      AutoBuilder.pathfindToPose(
        desiredTag2d.transformBy(Vision.reefAlignFar),
        new PathConstraints(
          Vision.reefInMaxVelocity,
          Vision.reefMaxAcceleration,
          Vision.reefMaxRotationalRate,
          Vision.reefMaxAccelerationRotationalRate
        ),
        0.0
      ), // move out
        new SequentialCommandGroup(
          new WaitCommand(0.2), // wait
          intakeOff() // intake off
        )
      )
    ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }
  
  public Command closestProcessorAlign() {
    // patented D.N.K.R G.A.V.I.N (Do Not Kill Robot - General Autonomous Vision Information Networking)
    Optional<Integer> closestTagOptional = vision.getClosestTagId(Vision.processorIds);
    if (closestTagOptional.isEmpty()) return new Command() {};

    int closestTag = closestTagOptional.get();
    final Pose2d desiredTag2d = vision.getTagPose2d(closestTag);
    

    Command align = AutoBuilder.pathfindToPose(
      desiredTag2d.transformBy(Vision.reefAlign),
      new PathConstraints(
        Vision.reefMaxVelocity,
        Vision.reefMaxAcceleration,
        Vision.reefMaxRotationalRate,
        Vision.reefMaxAccelerationRotationalRate
      ),
      0.0
    );


    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        AutoBuilder.pathfindToPose(
      desiredTag2d.transformBy(Vision.reefAlignFar),
      new PathConstraints(
        Vision.reefInMaxVelocity,
        Vision.reefMaxAcceleration,
        Vision.reefMaxRotationalRate,
        Vision.reefMaxAccelerationRotationalRate
      ),
      0.0
    ), // align to tag, but far away so shooter doesnt hit
        runProcessor() // to processor thing
      ), 
      align, // go in so shooter intakes ball
      shooter.runShootAlgae(), // shoot algae in
      AutoBuilder.pathfindToPose(
      desiredTag2d.transformBy(Vision.reefAlignFar),
      new PathConstraints(
        Vision.reefInMaxVelocity,
        Vision.reefMaxAcceleration,
        Vision.reefMaxRotationalRate,
        Vision.reefMaxAccelerationRotationalRate
      ),
      0.0
    ) // leave lol
    ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }
  
  /*
   * Aligns to the barge tag along a line.
   */
  public Command closestBargeAlign() {
    // patented D.N.K.R G.A.V.I.N (Do Not Kill Robot - General Autonomous Vision Information Networking)

    int closestTag = Vision.bargeAllianceMap.get(DriverStation.getAlliance().get());
    final Pose2d desiredTag2d = vision.getTagPose2d(closestTag).plus(Vision.bargeAlign);
    final double poseY = desiredTag2d.getY() - drivetrain.getState().Pose.getY();
    
    if (!vision.withinBarge(poseY)) return new Command() {}; // If out of barge length, do not align 
    
    final Pose2d desiredPosition = desiredTag2d.plus(new Transform2d(0, poseY, new Rotation2d())); // Transform so robot is along line

    Command align = AutoBuilder.pathfindToPose(
      desiredTag2d,
      new PathConstraints(
        Vision.bargeMaxVelocity,
        Vision.bargeMaxAcceleration,
        Vision.reefMaxRotationalRate,
        Vision.reefMaxAccelerationRotationalRate
      ),
      0.0
    );

    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        align, // align to tag
        runBarge() // go to barge shooting position
      ), 
      shooter.runShootAlgae() // shoot yippee!!
    ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public void setSafety(boolean state) {
    Validator.safetyEnabled = state;
  }

  private double getSwerveSpeedFraction() {
    // https://www.desmos.com/calculator/ghdph63sxf
    final double slowingHeight = 0;
    final double minSpeedFraction = 0.30;
    final double curveCoefficient = (minSpeedFraction - 1) / Math.pow(slowingHeight - 1, 2);
    double height =
        MathUtil.clamp(
            (elevator.getHeight() + 0.1 * Math.sin(shooter.getArmAngle() * 2 * Math.PI)) * 1.2, 0, 1);
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
   public void addVisionMeasurement() {
    Pose2d visionEstimate = vision.getEstimatedPose2d();
    if (visionEstimate == null) return; 
    Pose2d currentEstimate = drivetrain.getState().Pose;
    double estimatesDistance = currentEstimate.getTranslation().getDistance(visionEstimate.getTranslation());
    // Filter out bad vision measurements
    if (estimatesDistance > 1.0) return;
    
    drivetrain.addVisionMeasurement(visionEstimate, Timer.getFPGATimestamp());
  }
}
