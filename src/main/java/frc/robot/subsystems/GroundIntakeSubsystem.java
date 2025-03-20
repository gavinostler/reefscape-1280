package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
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

    GroundIntake.intakePID.setSetpoint(GroundIntake.UP_ANGLE);
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
   * Check if the current angle is at the state angle
   *
   * @return true if the current angle is at or past the given angle
   */
  public boolean atSetpoint() {
    return GroundIntake.intakePID.atSetpoint();
  }

  private void intakeUp() {
    GroundIntake.intakePID.setSetpoint(GroundIntake.UP_ANGLE);
  }

  private void intakeDown() {
    GroundIntake.intakePID.setSetpoint(GroundIntake.DOWN_ANGLE);
  }

  private void intakeOff() {
    disabled = true;
    intakeMotor.setVoltage(0.0);
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
  }

  public Command kickUp(double seconds) {
    return new SequentialCommandGroup(
      runOnce(() -> {
        intakeMotor.setVoltage(10.0);
        disabled = true;
        System.out.println("Applied impulse");
      }),
      new WaitCommand(seconds),
      runOnce(() -> {
        disabled = false;
      })
    );
  }

  @Override
  public void periodic() {
    // if (disabled) return;
    double voltage = GroundIntake.intakePID.calculate(getAngle()) + GroundIntake.intakeFf.calculate(state.angle * Math.PI * 2, 3.0);
    intakeMotor.setVoltage(voltage);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Ground Intake");
    builder.addDoubleProperty(
        "intake voltage",
        () -> intakeMotor.getAppliedOutput(),
        intakeMotor::setVoltage);
    builder.addDoubleProperty("angle", this::getAngle, null);
    builder.addDoubleProperty("setpoint", GroundIntake.intakePID::getSetpoint, null);
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
