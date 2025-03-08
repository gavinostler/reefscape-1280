package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Elevator;

public class ElevatorSubsystem implements Subsystem, Sendable {
  private final TalonFX motor = new TalonFX(Elevator.motorId);
  private final CANcoder encoder = new CANcoder(Elevator.encoderId);

  private final MotionMagicVoltage heightRequest =
      new MotionMagicVoltage(0.0).withFeedForward(Elevator.FF_TERM);
  private final VoltageOut movementRequest = new VoltageOut(0.0);

  private double goal = 0.0;
  private double heightOffset;

  public ElevatorSubsystem() {
    motor.getConfigurator().apply(Elevator.elevatorConfigs);
    heightOffset = getHeight(); // Assume elevator starts at bottom
  }

  public void moveToHeight(double fraction) {
    goal = MathUtil.clamp(fraction, 0.0, 1.0);
    motor.setControl(heightRequest.withPosition(goal));
  }

  public void moveToL1() {
    moveToHeight(Elevator.L1_HEIGHT_FRACTION);
  }

  public void moveToL2() {
    moveToHeight(Elevator.L2_HEIGHT_FRACTION);
  }

  public void stowElevator() {
    moveToHeight(Elevator.STOW_HEIGHT_FRACTION);
  }

  public void moveElevator(boolean downward) {
    motor.setControl(movementRequest.withOutput(Elevator.MOVEMENT_VOLTAGE * (downward ? -1 : 1)));
  }

  public void holdHeight() {
    moveToHeight(getHeight());
  }

  public void coast() {
    var configs = Elevator.elevatorConfigs;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motor.getConfigurator().apply(configs);
  }

  public void brake() {
    var configs = Elevator.elevatorConfigs;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motor.getConfigurator().apply(configs);
  }

  public double getEncoderHeight() {
    return encoder.getPosition().getValueAsDouble();
  }

  /**
   * @return Height fraction, from 0.0 to 1.0
   */
  public double getHeight() {
    // TODO: USES MOTOR INSTEAD OF CANCODER CURRENTLY
    System.out.println(
        "HEIGHT FRACTION: "
            + (motor.getRotorPosition().getValueAsDouble() / 20 / Elevator.HEIGHT_IN_ROTATIONS
                - heightOffset));
    return motor.getRotorPosition().getValueAsDouble() / 20 / Elevator.HEIGHT_IN_ROTATIONS
        - heightOffset;
  }

  public boolean atGoal() {
    return Math.abs(getHeight() - goal) < Elevator.TOLERANCE;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Elevator Carriage");
    builder.addDoubleProperty("true height", this::getHeight, null);
    builder.addDoubleProperty("goal height", () -> goal, this::moveToHeight); // dangerous
    builder.addBooleanProperty("within tolerance?", this::atGoal, null);

    builder.addDoubleProperty("other height, actual encoder", this::getEncoderHeight, null);
  }
}
