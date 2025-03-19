package frc.robot.state;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Validator implements Sendable {
  public ElevatorSubsystem elevator;
  public ShooterSubsystem shooter;
  public GroundIntakeSubsystem groundIntake;

  // Set to false to consider both state and position as always valid
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

  public boolean currentPositionsValid() {
    return positionsValid(
        elevator.getHeight(),
        shooter.getArmAngle(),
        groundIntake.getState() == State.GroundIntake.UP);
  }

  public boolean moveHeightValid(double height) {
    return positionsValid(
        height, shooter.getArmAngle(), groundIntake.getState() == State.GroundIntake.UP);
  }

  public boolean moveArmAngleValid(double angle) {
    return positionsValid(
        elevator.getHeight(), angle, groundIntake.getState() == State.GroundIntake.UP);
  }

  public boolean moveGroundIntakeValid(boolean up) {
    return positionsValid(elevator.getHeight(), shooter.getArmAngle(), up);
  }

  /**
   * Check if the given positions are physically valid and safe
   *
   * @param height elevator height fraction
   * @param armAngle shooter arm angle in rotations
   * @param intakeUp whether ground intake is up instead of down
   * @return true if it is a valid robot position, false if invalid or unsafe
   */
  public static boolean positionsValid(double height, double armAngle, boolean intakeUp) {
    if (!safetyEnabled) return true;
    if (height < 0.0 || height > 1.0) return false;
    if (armAngle < Constants.Shooter.ARM_MIN_ANGLE || armAngle > Constants.Shooter.ARM_MAX_ANGLE)
      return false;
    // TODO: find better values
    if (height > 0.6)
      // Elevator high enough for complete safety
      return true;
    if (intakeUp) {
      if (height > 0.3 && armAngle > 0.01)
        // Arm above horizontal and elevator high enough
        return true;
      if (armAngle > 0.21)
        // Arm stowed
        return true;
    } else {
      if (height > 0.1 && armAngle > 0.01)
        // Arm above horizontal and elevator high enough
        return true;
      if (armAngle > 0.1)
        // Elevator at bottom but arm high enough
        return true;
    }
    System.out.println(
        "(debug) Invalid positions: elevator="
            + height
            + ", arm="
            + armAngle
            + ", intakeUp="
            + intakeUp);
    return false;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Validator");
    builder.addStringProperty(
        "current positions",
        () ->
            "elevator="
                + elevator.getHeight()
                + ", shooter="
                + shooter.getArmAngle()
                + ", groundIntake="
                + groundIntake.getState().toString(),
        null);
    builder.addBooleanProperty("positions valid", this::currentPositionsValid, null);
    builder.addBooleanProperty(
        "SAFETY ENABLED",
        () -> safetyEnabled,
        (boolean value) -> {
          safetyEnabled = value;
        });
  }
}
