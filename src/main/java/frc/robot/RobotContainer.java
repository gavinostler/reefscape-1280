// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Driver;
// import frc.robot.aesthetic.Colors;
import frc.robot.controller.AgnosticController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

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
  private double MaxAngularRate =
      RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 3/4 of a rotation per second max
  // angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.Velocity); // Use open-loop control for drive motors

  // The robot's subsystems and commands are defined here...
  private final CommandSwerveDrivetrain drivetrain; // declared later due to NamedCommands
  private final GroundIntakeSubsystem groundIntake = new GroundIntakeSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  public final ElevatorSubsystem elevator = new ElevatorSubsystem();
  //   private final ClimberSubsystem climber = new ClimberSubsystem();
  private final AgnosticController driverController =
      new AgnosticController(Driver.kDriverControllerPort);
  private final AgnosticController operatorController =
      driverController; // Aliased to main controller for now, TODO
  private final Telemetry logger = new Telemetry(MaxSpeed);
  //   private final Colors colors = new Colors();

  // sendable for choosing autos
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    // colors.animateCandle(Colors.Effect.CHROMA);
    registerNamedCommands();
    drivetrain = TunerConstants.createDrivetrain(); // AFTER NamedCommands are registered
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser("ventura_auto");
    SmartDashboard.putData("Auto Mode", autoChooser);
    SmartDashboard.putData("shooter", shooter);
    SmartDashboard.putData("elevator", elevator);
    elevator.setSubsystems(shooter, groundIntake);
    shooter.setSubsystems(elevator, groundIntake);
  }

  /** Register named commands, for use in autonomous */
  public void registerNamedCommands() {
    // GroundIntake commands
    NamedCommands.registerCommand(
        "groundIntakeDown",
        groundIntake.runOnce(() -> groundIntake.setMode(GroundIntakeSubsystem.Mode.DOWN)));
    NamedCommands.registerCommand(
        "groundIntakeUp",
        groundIntake.runOnce(() -> groundIntake.setMode(GroundIntakeSubsystem.Mode.UP)));
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
    NamedCommands.registerCommand("shootProcessor", shooter.runShootProcessor());

    // Elevator commands
    NamedCommands.registerCommand(
        "moveElevatorL1", elevator.runOnce(elevator::moveToL1));
    NamedCommands.registerCommand(
        "moveElevatorL2", elevator.runOnce(elevator::moveToL2));
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
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(driverController.getLeftY() * MaxSpeed * (1 - elevator.getHeight()))
                    .withVelocityY(driverController.getLeftX() * MaxSpeed * (1 - elevator.getHeight()))
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate * (1 - elevator.getHeight()))));

    driverController
        .resetHeading()
        .onTrue(drivetrain.runOnce(() -> drivetrain.resetRotation(Rotation2d.kZero)));

    drivetrain.registerTelemetry(logger::telemeterize);

    // Operator Controls
    // operatorController.povUp().and(() ->
    // !climber.climberAtMax()).onTrue(climber.runOnce(climber::enable));
    // TODO: control to disable climber

    // Elevator controls
    operatorController.b().onTrue(elevator.runOnce(elevator::moveToL1));
    operatorController.y().onTrue(elevator.runOnce(elevator::moveToL2));
    operatorController
        .povUp()
        .whileTrue(elevator.runOnce(() -> elevator.moveElevator(false)))
        .whileTrue(elevator.run(elevator::elevatorMovement))
        .onFalse(elevator.runOnce(elevator::holdHeight));
    operatorController
        .povDown()
        .whileTrue(elevator.runOnce(() -> elevator.moveElevator(true)))
        .whileTrue(elevator.run(elevator::elevatorMovement))
        .onFalse(elevator.runOnce(elevator::holdHeight));

    // GroundIntake controls
    operatorController.a()
        .onTrue(groundIntake.runOnce(groundIntake::toggleMode))
        .onFalse(groundIntake.runOnce(groundIntake::off));

    // Shooter pivot controls
    operatorController.povRight()
        .onTrue(shooter.runOnce(() -> shooter.changePresetAngle(false)))
        .onFalse(shooter.runOnce(shooter::holdTargetArmAngle));
    operatorController
        .povLeft()
        .whileTrue(shooter.runOnce(() -> shooter.changePresetAngle(true)))
        .onFalse(shooter.runOnce(shooter::holdTargetArmAngle));

    // Shooter motor controls
    operatorController.leftTrigger()
        .onTrue(shooter.runOnce(() -> { shooter.intakeAlgae(); groundIntake.enablePulley(); }))
        .onFalse(shooter.runOnce(() -> { shooter.disableShooter(); shooter.brakeFeed(); groundIntake.disablePulley(); }));
    operatorController.rightTrigger().onTrue(shooter.runShootAlgae());
    operatorController.x().onTrue(shooter.runShootProcessor());

    // Reset odometry
    operatorController.resetHeading().onTrue(drivetrain.runOnce(() -> {
        Pose2d x = drivetrain.getState().Pose;
        x = x.rotateBy(new Rotation2d(Math.PI));
        drivetrain.resetPose(x);
    }));

    // Stow all subsystems
    operatorController.rightStick()
    .onTrue(shooter.runOnce(() -> { shooter.stowArm(); elevator.stowElevator(); new WaitCommand(1.2); groundIntake.setMode(GroundIntakeSubsystem.Mode.UP); }));
  }

  void bindSysId() {
    driverController //
        .back()
        .and(driverController.y())
        .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driverController //
        .back()
        .and(driverController.x())
        .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driverController //
        .start()
        .and(driverController.y())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driverController //
        .start()
        .and(driverController.x())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    driverController //
        .start()
        .and(driverController.povDown())
        .onTrue(drivetrain.runOnce(drivetrain::sysIdCycleRoutine));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
