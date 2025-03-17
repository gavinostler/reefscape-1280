package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Elevator;
import frc.robot.state.State;
import frc.robot.state.Validator;

public class ElevatorSubsystem implements Subsystem, Sendable {
  private final TalonFX motor = new TalonFX(Elevator.motorId);

  private final MotionMagicVoltage heightRequest =
      new MotionMagicVoltage(0.0).withFeedForward(Elevator.FF_TERM);

  private double targetHeight = 0.0;

  private State.Elevator state = State.Elevator.BOTTOM;
  private final Validator validator;

  public ElevatorSubsystem(Validator validator) {
    this.validator = validator;

    motor.getConfigurator().apply(Elevator.elevatorConfigs);

    setState(state);
  }

  public State.Elevator getState() {
    return state;
  }

  /**
   * Attempt to go to the given state If the state is currently impossible or is already the current
   * state, do nothing
   *
   * @param value the state to try to set
   */
  public void setState(State.Elevator value) {
    if (state == value || !validator.setStateValid(value)) return;
    state = value;
    moveHeight(state.height);
  }

  private State.Elevator nextState(boolean downward) {
    State.Elevator[] values = State.Elevator.values();
    int index = state.ordinal();
    index += downward ? 1 : -1;
    index = MathUtil.clamp(index, 0, values.length - 1);
    return values[index];
  }

  public void moveState(boolean downward) {
    setState(nextState(downward));
  }

  private void moveHeight(double height) {
    height = MathUtil.clamp(height, 0.0, 1.0);
    if (!validator.moveHeightValid(height)) {
      return;
    }
    targetHeight = height;
    motor.setControl(heightRequest.withPosition(targetHeight));
  }

  public void coast() {
    TalonFXConfiguration configs = Elevator.elevatorConfigs;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motor.getConfigurator().apply(configs);
  }

  public void brake() {
    TalonFXConfiguration configs = Elevator.elevatorConfigs;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motor.getConfigurator().apply(configs);
  }

  /**
   * @return Height fraction, approximately in the range of 0.0 to 1.0
   */
  public double getHeight() {
    // Note: Robot must start with elevator exactly at bottom for method to be work
    return motor.getPosition().getValueAsDouble();
  }

  /**
   * @return Returns true if elevator height is within tolerance of the state setpoint
   */
  public boolean atSetpoint() {
    return MathUtil.isNear(state.height, getHeight(), Elevator.HEIGHT_TOLERANCE);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Elevator");
    builder.addDoubleProperty("true height", this::getHeight, null);
    builder.addDoubleProperty("target height", () -> targetHeight, this::moveHeight);
    builder.addDoubleProperty(
        "rotor position", () -> motor.getRotorPosition().getValueAsDouble(), null);
    builder.addStringProperty("State", () -> getState().toString(), null);
  }
}
