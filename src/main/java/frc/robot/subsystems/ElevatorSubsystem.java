package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Elevator;

public class ElevatorSubsystem implements Subsystem, Sendable {
    private final TalonFX motor = new TalonFX(Elevator.motorId);
    private final Encoder encoder = new Encoder(1,2);

    private final MotionMagicVoltage heightRequest =
        new MotionMagicVoltage(0.0).withFeedForward(Elevator.FF_TERM);
    private final VoltageOut movementRequest = new VoltageOut(0.0);
    private double goal = 0.0;

    public ElevatorSubsystem() {
        motor.getConfigurator().apply(Elevator.elevatorConfigs);
        encoder.reset(); // assuming elevator starts at bottom
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
        getHeight();
        motor.setControl(movementRequest.withOutput(Elevator.MOVEMENT_VOLTAGE * (downward ? -1 : 1)));
    }

    public void keepHeight() {
        motor.setControl(new NeutralOut());
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

    public double getEncoderHeight(){
        return encoder.get();
    }

    /**
     * @return Height fraction, from 0.0 to 1.0
     */
    public double getHeight() {
        double height = (encoder.get() /  Elevator.ELEVATOR_HEIGHT);
        System.out.println("HEIGHT FRACTION: " + height);
        return height;
    }

  public boolean atGoal() {
    return Math.abs(getHeight() - goal) < Elevator.TOLERANCE;
  }

    // Send values to dashboard
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Elevator Carriage");
        builder.addDoubleProperty("true height", this::getHeight, null);
        builder.addDoubleProperty("goal height", () -> goal, this::moveToHeight); // dangerous
        builder.addBooleanProperty("within tolerance?", this::atGoal, null);

    builder.addDoubleProperty("other height, actual encoder", this::getEncoderHeight, null);
  }
}
