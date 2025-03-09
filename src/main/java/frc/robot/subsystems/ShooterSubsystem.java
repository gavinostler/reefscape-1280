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
import frc.robot.Constants.Shooter;

public class ShooterSubsystem implements Subsystem, Sendable {
  // Motors and encoders
  // private final CANdi beamBreak = new CANdi(Shooter.beamBreakId);
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
  private final VoltageOut armMovementRequest = new VoltageOut(0.0);

  private double targetAngle;

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

  // public boolean algaeInFeed() {
  //   return beamBreak.getS1State().getValue() == com.ctre.phoenix6.signals.S1StateValue.High;
  // }

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

  /**
   * Zero is horizontal
   *
   * @param angle Rotations
   */
  public void setArmAngle(double angle) {
    angle = Math.min(angle, Shooter.ARM_MAX_ROTATION);
    angle = Math.max(angle, Shooter.ARM_MIN_ROTATION);
    armMotor.setControl(armAngleRequest.withPosition(angle));
    targetAngle = angle;
  }

  /** Set arm target angle to current arm angle */
  public void holdArmAngle() {
    setArmAngle(getArmAngle());
  }

  /**
   * @param downward false for up, true for down
   */
  public void moveArm(boolean downward) {
    double voltage = downward ? -Shooter.ARM_VOLTAGE_DOWN : Shooter.ARM_VOLTAGE_UP;
    armMotor.setControl(armMovementRequest.withOutput(voltage));
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
