package frc.robot.state;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Validator implements Sendable {
  public ElevatorSubsystem elevator;
  public ShooterSubsystem shooter;
  public GroundIntakeSubsystem groundIntake;

  // Set to false to consider state as always valid
  public static boolean safetyEnabled = true;

  /**
   * Check if setting a state is valid
   *
   * @param state any proposed subsystem state to check
   * @return true if it is valid to set, false if invalid
   */
  public boolean setStateValid(State.Elevator state) {
    return !safetyEnabled || State.isValid(state, shooter.getState(), groundIntake.getState());
  }

  public boolean setStateValid(State.Shooter state) {
    return !safetyEnabled || State.isValid(elevator.getState(), state, groundIntake.getState());
  }

  public boolean setStateValid(State.GroundIntake state) {
    return !safetyEnabled || State.isValid(elevator.getState(), shooter.getState(), state);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Validator");
    builder.addBooleanProperty(
        "SAFETY ENABLED",
        () -> safetyEnabled,
        (boolean value) -> {
          safetyEnabled = value;
        });
  }
}
