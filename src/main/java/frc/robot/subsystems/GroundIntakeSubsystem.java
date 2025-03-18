package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.GroundIntake;
import frc.robot.state.State;
import frc.robot.state.Validator;

public class GroundIntakeSubsystem extends SubsystemBase {
  private final TalonFX pulleyMotor = new TalonFX(GroundIntake.pulleyId);
  private final SparkMax intakeMotor = new SparkMax(GroundIntake.intakeId, MotorType.kBrushless);
  private final Encoder encoder =
      new Encoder(GroundIntake.encoderChannelA, GroundIntake.encoderChannelB);

  private double intakeVoltage = 0.0;
  private boolean disabled = false;

  private State.GroundIntake state = State.GroundIntake.UP;
  private final Validator validator;

  public GroundIntakeSubsystem(Validator validator) {
    this.validator = validator;

    pulleyMotor.getConfigurator().apply(GroundIntake.pulleyConfigs);
    intakeMotor.configure(
        GroundIntake.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder.reset();
    encoder.setDistancePerPulse(GroundIntake.DISTANCE_PER_PULSE);
    encoder.setReverseDirection(GroundIntake.REVERSE_ENCODER);

    // setState(state);
  }

  public State.GroundIntake getState() {
    return state;
  }

  /**
   * Attempt to go to the given state If the state is currently impossible or is already the current
   * state, do nothing
   *
   * @param value the state to try to set
   */
  public void setState(State.GroundIntake value) {
    if (!validator.setStateValid(value)) return;
    state = value;
    switch (state) {
      case DOWN -> down();
      case UP -> up();
    }
  }

  private State.GroundIntake nextState() {
    return switch (state) {
      case DOWN -> State.GroundIntake.UP;
      case UP -> State.GroundIntake.DOWN;
    };
  }

  public void toggleState() {
    setState(nextState());
  }

  /**
   * Get the current ground intake angle, where 0 is the horizontal, and positive is up
   *
   * @return angle in rotations
   */
  public double getAngle() {
    return encoder.getDistance() + GroundIntake.ENCODER_OFFSET;
  }

  /**
   * Check if the current angle is at or past a given angle This factors in the direction of the
   * intake motor
   *
   * @param targetAngle the angle to see if the ground intake is at or past
   * @return true if the current angle is at or past the given angle
   */
  public boolean atAngle(double targetAngle) {
    if (intakeVoltage != 0.0)
      return Math.signum(intakeVoltage) * (targetAngle - getAngle()) < GroundIntake.ANGLE_TOLERANCE;
    return Math.abs(targetAngle - getAngle()) < GroundIntake.ANGLE_TOLERANCE;
  }

  private void setIntakeVoltage(double voltage) {
    intakeVoltage = voltage;
    intakeMotor.setVoltage(intakeVoltage);
  }

  private void intakeUp() {
    // setIntakeVoltage(GroundIntake.INTAKE_UP_VOLTAGE);
    GroundIntake.intakePID.setSetpoint(GroundIntake.UP_ANGLE);
    new SequentialCommandGroup(
      runOnce(() -> {
        setIntakeVoltage(5);
        disabled = true;
      }),
      new WaitCommand(0.3),
      runOnce(() -> {
        disabled = false;
      })
    ).schedule();
  }

  private void intakeDown() {
    // setIntakeVoltage(GroundIntake.INTAKE_DOWN_VOLTAGE);
    GroundIntake.intakePID.setSetpoint(GroundIntake.DOWN_ANGLE);
  }

  private void intakeOff() {
    setIntakeVoltage(0.0);
  }

  public void enablePulley() {
    pulleyMotor.setVoltage(GroundIntake.PULLEY_VOLTAGE);
  }

  public void disablePulley() {
    pulleyMotor.setVoltage(0.0);
  }

  private void up() {
    if (!validator.moveGroundIntakeValid(true)) return;
    intakeUp();
  }

  private void down() {
    if (!validator.moveGroundIntakeValid(false)) return;
    intakeDown();
  }

  public void off() {
    intakeOff();
    disablePulley();
    disabled = true;
  }

  @Override
  public void periodic() {
    // if (intakeVoltage != 0.0 && atAngle(state.angle)) intakeOff();
    if (disabled) return;
    setIntakeVoltage(
      GroundIntake.intakePID.calculate(getAngle()) + 
      GroundIntake.intakeFf.calculate(state.angle, 1.0)
    );
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Ground Intake");
    builder.addDoubleProperty(
        "pulley voltage",
        () -> pulleyMotor.getMotorVoltage().getValueAsDouble(),
        pulleyMotor::setVoltage);
    builder.addDoubleProperty("intake voltage", () -> intakeVoltage, this::setIntakeVoltage);
    builder.addDoubleProperty("angle", this::getAngle, null);
    builder.addBooleanProperty("at state angle", () -> atAngle(state.angle), null);
    builder.addStringProperty(
        "State",
        () -> getState().toString(),
        (String value) -> {
          State.GroundIntake state;
          try {
            state = State.GroundIntake.valueOf(value.toUpperCase());
          } catch (IllegalArgumentException e) {
            return;
          }
          setState(state);
        });
  }
}
