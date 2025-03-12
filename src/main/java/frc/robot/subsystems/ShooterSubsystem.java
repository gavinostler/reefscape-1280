package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.GroundIntakeSubsystem.Mode;

public class ShooterSubsystem implements Subsystem, Sendable {
  // Motors and encoders
  private final TalonFX leaderShooterMotor = new TalonFX(Shooter.rightShooterId);
  private final TalonFX followerShooterMotor = new TalonFX(Shooter.leftShooterId);
  private final SparkMax leaderFeedMotor = new SparkMax(Shooter.rightFeedId, MotorType.kBrushless);
  private final SparkMax followerFeedMotor = new SparkMax(Shooter.leftFeedId, MotorType.kBrushless);
  private final TalonFX armMotor = new TalonFX(Shooter.armId);
  private final CANcoder armEncoder = new CANcoder(Shooter.armEncoderId);

  private final NeutralOut shooterNeutralRequest = new NeutralOut();
  private final VelocityVoltage shooterVelocityRequest =
      new VelocityVoltage(0.0).withSlot(0).withFeedForward(Shooter.SHOOTER_FF_TERM);
  private final MotionMagicVoltage armAngleRequest =
      new MotionMagicVoltage(0.0).withFeedForward(Shooter.ARM_FF_TERM);
  private final VelocityVoltage armVelocityRequest =
      new VelocityVoltage(0.0).withSlot(1).withFeedForward(Shooter.ARM_FF_TERM);

  private double targetAngle;

  private int currentPreset = Shooter.ARM_POSITIONS.length - 1;
  private ElevatorSubsystem elevator;
  private GroundIntakeSubsystem intake;

  public ShooterSubsystem() {
    leaderShooterMotor.getConfigurator().apply(Shooter.shooterConfigs);
    followerShooterMotor.getConfigurator().apply(Shooter.shooterConfigs);
    followerShooterMotor.setControl(new Follower(Shooter.rightShooterId, true));

    leaderFeedMotor.configure(
        Shooter.leaderFeedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followerFeedMotor.configure(
        Shooter.followerFeedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    armMotor.getConfigurator().apply(Shooter.armConfigs);
  }

  public void setSubsystems(ElevatorSubsystem elevator, GroundIntakeSubsystem intake) {
    this.elevator = elevator;
    this.intake = intake;
  }

  public void changePresetAngle(boolean inward) {
    // terrible way of changing dpad position because im too lazy to make it look nice
    if((inward && currentPreset == 0) || (!inward && currentPreset == Shooter.ARM_POSITIONS.length - 1)) {
      return;
    }
    int lastPreset = currentPreset;
    currentPreset = inward ? currentPreset - 1 : currentPreset + 1;
    if (!setArmAngle(Shooter.ARM_POSITIONS[currentPreset])) {
      currentPreset = lastPreset;
    };
  }

  /**
   * @param inward Sets the shooter direction for intake/shooting. (it sucks)
   */
  public void enableShooter(boolean inward) {
    double target = inward ? -Shooter.SHOOTER_INTAKE_TARGET_RPS : Shooter.SHOOTER_SHOOT_TARGET_RPS;
    leaderShooterMotor.setControl(
        shooterVelocityRequest.withVelocity(Shooter.SHOOTER_GEAR_REDUCTION * target));
  }

  // ugly method for processor
  public void shootProcessor() {
    double target = Shooter.SHOOTER_PROCESSOR_TARGET_RPS;
    leaderShooterMotor.setControl(
        shooterVelocityRequest.withVelocity(Shooter.SHOOTER_GEAR_REDUCTION * target));
  }

  public void disableShooter() {
    leaderShooterMotor.setControl(shooterNeutralRequest);
  }

  /**
   * @param inward Sets the feed direction for intake/shooting. (it sucks)
   */
  public void enableFeed(boolean inward) {
    leaderFeedMotor.set(Shooter.FEED_SPEED * (inward ? -1 : 1));
    // leaderFeedMotor.getClosedLoopController().setReference(Shooter.FEED_TARGET_RPM,
    // ControlType.kVelocity);
  }

  /** Set the PID controller of feed to target current position */
  public void brakeFeed() {
    double currentPosition = leaderFeedMotor.getEncoder().getPosition();
    leaderFeedMotor.getClosedLoopController().setReference(currentPosition, ControlType.kPosition);
  }

  /** Disables the feed motor */
  public void disableFeed() {
    leaderFeedMotor.set(0.0);
  }

  public void disableShooterAndFeed() {
    disableFeed();
    disableShooter();
  }

  /**
   * Get arm angle from encoder
   *
   * @return Angle in rotations
   */
  public double getArmAngle() {
    return armEncoder.getPosition().getValueAsDouble();
  }

  public boolean safeToMove(double proposedAngle) {
    if (elevator ==  null) return false;

    if (elevator.getHeight() < Elevator.SAFETY_GENERAL_HEIGHT && proposedAngle < 0.02 && intake.getMode() == Mode.DOWN) return false;
    if (elevator.getHeight() < Elevator.SAFETY_ZERO_HEIGHT && intake.getMode() == Mode.UP) return false;

    return  true;
  }

  /**
   * Zero is horizontal
   *
   * @param angle Rotations
   * @return boolean if successful
   */
  public boolean setArmAngle(double angle) {
    angle = Math.min(angle, Shooter.ARM_MAX_ROTATION);
    angle = Math.max(angle, Shooter.ARM_MIN_ROTATION);

    if (!safeToMove(angle)) return false;
    armMotor.setControl(armAngleRequest.withPosition(angle));
    targetAngle = angle;
    return true;
  }

  /** Set arm target angle to current arm angle */
  public void holdArmAngle() {
    setArmAngle(getArmAngle());
  }

  public void holdTargetArmAngle() {
    setArmAngle(targetAngle);
  }

  /**
   * @param downward false for up, true for down
   */
  public void moveArm(boolean downward) {
    double velocity = downward ? -Shooter.ARM_VELOCITY_DOWN : Shooter.ARM_VELOCITY_UP;
    armMotor.setControl(armVelocityRequest.withVelocity(velocity));
  }

  // Method to stow arm at set angle
  public void stowArm() {
    setArmAngle(Shooter.ARM_STOW_ROTATION);
  }

  /** Method to intake algae by starting feed and shooter inwards */
  public void intakeAlgae() {
    enableFeed(true);
    enableShooter(true);
  }

  /**
   * Command group to shoot algae
   *
   * @return Command for the shoot procedure
   */
  public Command runShootAlgae() {
    return new SequentialCommandGroup(
            runOnce(this::disableShooterAndFeed), // stop everything first
            runOnce(() -> enableFeed(true)),
            runOnce(() -> enableShooter(true)), // pull algae back
            new WaitCommand(0.5), // time to pull back
            runOnce(() -> enableShooter(false)), // rev up the shooter
            new WaitCommand(0.5), // give it time to come up to target speed
            runOnce(() -> enableFeed(false)), // hawk tuah
            new WaitCommand(1.5), // wait for shooting to finish
            runOnce(this::disableShooterAndFeed) // turn everything off
            )
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming); // don't get interrupted
  }

  /**
   * Command group to shoot processor
   *
   * @return Command for the shoot procedure
   */
  public Command runShootProcessor() {
    return new SequentialCommandGroup(
            runOnce(this::disableShooterAndFeed), // stop everything first
            runOnce(() -> enableFeed(true)),
            new WaitCommand(1), // time to pull back
            runOnce(() -> shootProcessor()), // rev up the shooter to processor speed
            new WaitCommand(1.2), // give it time to come up to target speed
            runOnce(() -> enableFeed(false)), // hawk tuah
            new WaitCommand(1.5), // wait for shooting to finish
            runOnce(this::disableShooterAndFeed) // turn everything off
            )
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming); // don't get interrupted
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Shooter");
    builder.addDoubleProperty("true arm angle", this::getArmAngle, null);
    builder.addDoubleProperty("goal arm angle", () -> targetAngle, this::setArmAngle);
  }
}
