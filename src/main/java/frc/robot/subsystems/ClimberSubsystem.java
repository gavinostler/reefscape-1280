package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Climber;

import com.ctre.phoenix6.hardware.CANcoder;

public class ClimberSubsystem implements Subsystem {
    private final TalonFX m_motor = new TalonFX(Climber.id);
    private final CANcoder m_canCoder = new CANcoder(Climber.id);

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
        if (!enabled) {
            m_motor.set(0.0);
            return;
        }
        double speed;
        if (m_canCoder.getPosition().getValueAsDouble() + encoderOffset > Climber.maxValue) {
            speed = 0.0;
            enabled = false;
        } else {
            speed = Climber.kRotateSpeed;
        }
        m_motor.set(speed);
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
    }

}
