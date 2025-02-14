package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Climber;

public class ClimberSubsystem implements Subsystem {
  private final TalonFX m_motor = new TalonFX(Climber.id);
  private final CANcoder e_cancoder = new CANcoder(Climber.encoderId);

  private double encoderOffset = 0.0;

  private boolean enabled = false;

  public ClimberSubsystem() {
  }

  public Command runCommand() {
    return this.run(
        () -> {
          this.disable();
        });
  }

  public void rotate() {
    if (e_cancoder.getPosition().getValueAsDouble() + encoderOffset > Climber.maxValue) {
      disable();
    }
    double speed = enabled ? Climber.kRotateSpeed : 0.0;
    m_motor.set(speed);
  }

  public void enable() {
    enabled = true;
  }

  public void disable() {
    enabled = false;
  }
}
