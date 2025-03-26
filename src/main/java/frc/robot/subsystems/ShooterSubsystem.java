package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Shooter;
import frc.robot.state.State;
import frc.robot.state.Validator;

public class ShooterSubsystem implements Subsystem, Sendable {
  private final TalonFX leaderShooterMotor = new TalonFX(Shooter.rightShooterId);
  private final TalonFX followerShooterMotor = new TalonFX(Shooter.leftShooterId);
  private final SparkMax leaderFeedMotor = new SparkMax(Shooter.rightFeedId, MotorType.kBrushless);
  private final SparkMax followerFeedMotor = new SparkMax(Shooter.leftFeedId, MotorType.kBrushless);
  private final TalonFX armMotor = new TalonFX(Shooter.armId);
  private final CANcoder armEncoder = new CANcoder(Shooter.armEncoderId);

  private final VelocityVoltage shooterVelocityRequest = new VelocityVoltage(0.0);
  private final MotionMagicVoltage armAngleRequest = new MotionMagicVoltage(0.0);

  private double targetArmAngle = 0.0;

  private State.Shooter state = State.Shooter.STOW;
  private final Validator validator;

  public ShooterSubsystem(Validator validator) {
    this.validator = validator;

    leaderShooterMotor.getConfigurator().apply(Shooter.shooterConfigs);
    followerShooterMotor.getConfigurator().apply(Shooter.shooterConfigs);
    followerShooterMotor.setControl(new Follower(Shooter.rightShooterId, true));

    leaderFeedMotor.configure(
        Shooter.leaderFeedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followerFeedMotor.configure(
        Shooter.followerFeedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    armMotor.getConfigurator().apply(Shooter.armConfigs);
  }

  public State.Shooter getState() {
    return state;
  }

  /**
   * Attempt to go to the given state If the state is currently impossible or is already the current
   * state, do nothing
   *
   * @param value the state to try to set
   */
  public void setState(State.Shooter value) {
    if (!validator.setStateValid(value)) return;
    state = value;
    moveArmAngle(state.angle);
  }

  /**
   * Get the next state in the given direction, from the current state The state returned will be
   * the current state when there is no next state.
   *
   * @param downward false for up, true for down
   * @return the next closest state, or the current state
   */
  private State.Shooter nextState(boolean downward) {
    State.Shooter[] values = State.Shooter.values();
    int index = state.ordinal();
    index += downward ? 1 : -1;
    index = MathUtil.clamp(index, 0, values.length - 1);
    return values[index];
  }

  /**
   * Try to move the state in a certain direction.
   *
   * @param downward false for up, true for down
   */
  public void moveState(boolean downward) {
    setState(nextState(downward));
  }

  /**
   * @param inward Sets the shooter direction for intake/shooting. (it sucks)
   */
  public void enableShooter(boolean inward) {
    double target;
    if (getArmAngle() >= 0.0)
      target = inward ? Shooter.SHOOTER_INTAKE_TARGET_RPS : Shooter.SHOOTER_SHOOT_TARGET_RPS;
    else target = inward ? Shooter.SHOOTER_INTAKE_TARGET_RPS : Shooter.SHOOTER_PROCESSOR_TARGET_RPS;
    leaderShooterMotor.setControl(
        shooterVelocityRequest.withVelocity(Shooter.SHOOTER_GEAR_REDUCTION * target));
  }

  public void disableShooter() {
    leaderShooterMotor.setControl(shooterVelocityRequest.withVelocity(0.0));
  }

  /**
   * @param inward Sets the feed direction for intake/shooting. (it sucks)
   */
  public void enableFeed(boolean inward) {
    double speed = inward ? Shooter.FEED_IN_SPEED : Shooter.FEED_OUT_SPEED;
    leaderFeedMotor.set(speed);
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
   * @return Returns true if shooter angle is within tolerance of the state setpoint
   */
  public boolean atSetpoint() {
    return MathUtil.isNear(state.angle, getArmAngle(), Shooter.ANGLE_TOLERANCE);
  }

  /**
   * Zero is horizontal Will warn and not move if angle is outside arm range
   *
   * @param angle Rotations
   * @return boolean if successful
   */
  public void moveArmAngle(double angle) {
    targetArmAngle = angle;
    armMotor.setControl(armAngleRequest.withPosition(targetArmAngle));
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
            // runOnce(() -> enableFeed(true)), // ASSUME algae is already pulled back
            // runOnce(() -> enableShooter(true)), // pull algae back
            // new WaitCommand(0.2), // time to pull back
            runOnce(() -> enableShooter(false)), // rev up the shooter
            new WaitCommand(0.7), // give it time to come up to target speed
            runOnce(() -> enableFeed(false)), // hawk tuah
            new WaitCommand(1), // wait for shooting to finish
            runOnce(this::disableShooterAndFeed) // turn everything off
            )
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming); // don't get interrupted
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Shooter");
    builder.addDoubleProperty("true arm angle", this::getArmAngle, null);
    builder.addDoubleProperty("target arm angle", () -> targetArmAngle, this::moveArmAngle);
    builder.addStringProperty("State", () -> getState().toString(), null);
  }
}
