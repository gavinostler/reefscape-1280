// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.aesthetic.Colors;
import frc.robot.util.AgnosticController;

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
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric forwardStraight =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // The robot's subsystems and commands are defined here...
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final ClimberSubsystem climber = new ClimberSubsystem();
  private final Colors m_color = new Colors();
  // private final Music m_music = new Music();
  private final AgnosticController controller = new AgnosticController();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_color.colorStatic(199, 21, 133);

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.Commandcontroller Flight
   * controllers}.
   */
  public void configureBindings() {
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(controller.getLeftY() * MaxSpeed)
                    .withVelocityY(controller.getLeftX() * MaxSpeed)
                    .withRotationalRate(-controller.getRightX() * MaxAngularRate)));

    // Run forward, then reverse procedure to effectively "zero" characterization
    if (Constants.kEnableSysId) {
      controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
      controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
      controller
          .start()
          .and(controller.y())
          .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
      controller
          .start()
          .and(controller.x())
          .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

      controller
          .start()
          .and(controller.povDown())
          .onTrue(drivetrain.runOnce(drivetrain::sysIdCycleRoutine));
    }

    controller.resetHeading().onTrue(drivetrain.runOnce(() -> drivetrain.resetRotation(Rotation2d.kZero)));

    drivetrain.registerTelemetry(logger::telemeterize);
    climber.setDefaultCommand(climber.runCommand());
    controller.povUp().whileTrue(climber.runOnce(climber::enable));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
