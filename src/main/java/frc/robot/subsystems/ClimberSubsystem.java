package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Climber;
import frc.robot.Constants.GroundIntake;

public class ClimberSubsystem implements Subsystem {
  private final TalonFX motor;
  private final CANcoder encoder;

  private boolean enabled = false;

  public ClimberSubsystem() {
    motor = new TalonFX(Climber.motorId);
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimit = GroundIntake.INTAKE_CURRENT_LIMIT;
    motor.getConfigurator().apply(motorConfigs);
    encoder = new CANcoder(Climber.encoderId);
  }

  public double getAngle() {
    return encoder.getPosition().getValueAsDouble();
  }

  public boolean climberAtMax() {
    return getAngle() >= Climber.MAX_ANGLE;
  }

  public void enable() {
    enabled = true;
    motor.set(Climber.SPEED);
  }

  public void disable() {
    enabled = false;
    motor.set(0.0);
  }

  @Override
  public void periodic() {
    if (!enabled) return;
    if (climberAtMax()) disable();
  }
}
