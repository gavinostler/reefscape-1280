package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.GroundIntake;

public class GroundIntakeSubsystem implements Subsystem {
  public enum Mode {
    UP,
    DOWN,
  }

  private final TalonFX pulleyMotor;
  private final SparkMax intakeMotor;
  private Mode mode;

  public GroundIntakeSubsystem() {
    pulleyMotor = new TalonFX(GroundIntake.pulleyId);
    TalonFXConfiguration pulleyConfigs = new TalonFXConfiguration();
    pulleyConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    pulleyConfigs.CurrentLimits.StatorCurrentLimit = GroundIntake.PULLEY_CURRENT_LIMIT;
    pulleyMotor.getConfigurator().apply(pulleyConfigs);

    intakeMotor = new SparkMax(GroundIntake.intakeId, MotorType.kBrushless);
    intakeMotor.configure(
        GroundIntake.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    mode = Mode.UP; // Assumes up is the starting position
  }

  public Mode getMode() {
    return mode;
  }

  public void setMode(Mode value) {
    mode = value;
    SmartDashboard.putString("Intake mode", mode.toString());

    switch (mode) {
      case UP -> up();
      case DOWN -> down();
    }
  }

  public void toggleMode() {
    switch (mode) {
      case UP -> setMode(Mode.DOWN);
      case DOWN -> setMode(Mode.UP);
    }
  }

  public void intakeUp() {
    intakeMotor.setVoltage(GroundIntake.INTAKE_UP_VOLTAGE);
  }

  public void disablePulley() {
    pulleyMotor.setVoltage(0.0);
  }

  public void up() {
    intakeUp();
    disablePulley();
  }

  public void intakeDown() {
    intakeMotor.setVoltage(GroundIntake.INTAKE_DOWN_VOLTAGE);
  }

  public void enablePulley() {
    pulleyMotor.setVoltage(GroundIntake.PULLEY_VOLTAGE);
  }

  private void down() {
    intakeDown();
    enablePulley();
  }

  public void off() {
    pulleyMotor.setVoltage(0.0);
    intakeMotor.setVoltage(0.0);
  }
}
