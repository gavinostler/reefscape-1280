package frc.robot.state;

import frc.robot.Constants;
import java.util.HashMap;
import java.util.Map;

public class State {
  public static enum Elevator {
    // Highest to lowest
    TOP(Constants.Elevator.TOP_HEIGHT),
    SHOOT(Constants.Elevator.SHOOT_HEIGHT),
    L2(Constants.Elevator.L2_HEIGHT),
    GROUND_INTAKE(Constants.Elevator.GROUND_INTAKE_HEIGHT),
    L1(Constants.Elevator.L1_HEIGHT),
    BOTTOM(Constants.Elevator.BOTTOM_HEIGHT);

    public final double height;

    private Elevator(double height) {
      this.height = height;
    }
  }

  public static enum Shooter {
    // Arm only
    // Highest to lowest
    STOW(Constants.Shooter.STOW_ANGLE),
    SHOOT(Constants.Shooter.SHOOT_ANGLE),
    REEF_INTAKE(Constants.Shooter.REEF_INTAKE_ANGLE),
    GROUND_INTAKE(Constants.Shooter.GROUND_INTAKE_ANGLE),
    DOWN(-0.26),
    UP(0.26);

    public final double angle;

    private Shooter(double angle) {
      this.angle = angle;
    }
  }

  public static enum GroundIntake {
    DOWN(Constants.GroundIntake.DOWN_ANGLE),
    UP(Constants.GroundIntake.UP_ANGLE);

    public final double angle;

    private GroundIntake(double angle) {
      this.angle = angle;
    }
  }

  private static class RobotState {
    private final Elevator elevator;
    private final Shooter shooter;
    private final GroundIntake groundIntake;

    public RobotState(Elevator elevator, Shooter shooter, GroundIntake groundIntake) {
      this.elevator = elevator;
      this.shooter = shooter;
      this.groundIntake = groundIntake;
    }

    public boolean equals(Object obj) {
      if (!(obj instanceof RobotState)) {
        return false;
      }
      RobotState state = (RobotState) obj;
      return elevator == state.elevator
          && shooter == state.shooter
          && groundIntake == state.groundIntake;
    }

    public String toString() {
      return "[elevator="
          + elevator.toString()
          + ", shooter="
          + shooter.toString()
          + "groundIntake="
          + groundIntake.toString()
          + "]";
    }

    public int hashCode() {
      return (elevator.toString() + shooter.toString() + groundIntake.toString()).hashCode();
    }
  }

  private static Map<RobotState, Boolean> statesValid =
      new HashMap<RobotState, Boolean>() {
        // Cartesian product
        {
          put(new RobotState(Elevator.TOP, Shooter.STOW, GroundIntake.DOWN), true);
          put(new RobotState(Elevator.TOP, Shooter.STOW, GroundIntake.UP), true);
          put(new RobotState(Elevator.TOP, Shooter.SHOOT, GroundIntake.DOWN), true);
          put(new RobotState(Elevator.TOP, Shooter.SHOOT, GroundIntake.UP), true);
          put(new RobotState(Elevator.TOP, Shooter.REEF_INTAKE, GroundIntake.DOWN), true);
          put(new RobotState(Elevator.TOP, Shooter.REEF_INTAKE, GroundIntake.UP), true);
          put(new RobotState(Elevator.TOP, Shooter.GROUND_INTAKE, GroundIntake.DOWN), true);
          put(new RobotState(Elevator.TOP, Shooter.GROUND_INTAKE, GroundIntake.UP), true);

          put(new RobotState(Elevator.SHOOT, Shooter.STOW, GroundIntake.DOWN), true);
          put(new RobotState(Elevator.SHOOT, Shooter.STOW, GroundIntake.UP), true);
          put(new RobotState(Elevator.SHOOT, Shooter.SHOOT, GroundIntake.DOWN), true);
          put(new RobotState(Elevator.SHOOT, Shooter.SHOOT, GroundIntake.UP), true);
          put(new RobotState(Elevator.SHOOT, Shooter.REEF_INTAKE, GroundIntake.DOWN), true);
          put(new RobotState(Elevator.SHOOT, Shooter.REEF_INTAKE, GroundIntake.UP), true);
          put(new RobotState(Elevator.SHOOT, Shooter.GROUND_INTAKE, GroundIntake.DOWN), true);
          put(new RobotState(Elevator.SHOOT, Shooter.GROUND_INTAKE, GroundIntake.UP), true);

          put(new RobotState(Elevator.L2, Shooter.STOW, GroundIntake.DOWN), true);
          put(new RobotState(Elevator.L2, Shooter.STOW, GroundIntake.UP), true);
          put(new RobotState(Elevator.L2, Shooter.SHOOT, GroundIntake.DOWN), true);
          put(new RobotState(Elevator.L2, Shooter.SHOOT, GroundIntake.UP), true);
          put(new RobotState(Elevator.L2, Shooter.REEF_INTAKE, GroundIntake.DOWN), true);
          put(new RobotState(Elevator.L2, Shooter.REEF_INTAKE, GroundIntake.UP), true);
          put(new RobotState(Elevator.L2, Shooter.GROUND_INTAKE, GroundIntake.DOWN), true);
          put(new RobotState(Elevator.L2, Shooter.GROUND_INTAKE, GroundIntake.UP), true);
          // Above here, all robot states are always valid as the elevator is high enough

          put(new RobotState(Elevator.GROUND_INTAKE, Shooter.STOW, GroundIntake.DOWN), true);
          put(new RobotState(Elevator.GROUND_INTAKE, Shooter.STOW, GroundIntake.UP), true);
          put(new RobotState(Elevator.GROUND_INTAKE, Shooter.SHOOT, GroundIntake.DOWN), true);
          put(new RobotState(Elevator.GROUND_INTAKE, Shooter.SHOOT, GroundIntake.UP), true);
          put(new RobotState(Elevator.GROUND_INTAKE, Shooter.REEF_INTAKE, GroundIntake.DOWN), true);
          put(new RobotState(Elevator.GROUND_INTAKE, Shooter.REEF_INTAKE, GroundIntake.UP), true);
          // ground intake
          put(
              new RobotState(Elevator.GROUND_INTAKE, Shooter.GROUND_INTAKE, GroundIntake.DOWN),
              true);
          put(
              new RobotState(Elevator.GROUND_INTAKE, Shooter.GROUND_INTAKE, GroundIntake.UP),
              false);

          put(new RobotState(Elevator.L1, Shooter.STOW, GroundIntake.DOWN), true);
          put(new RobotState(Elevator.L1, Shooter.STOW, GroundIntake.UP), true);
          put(new RobotState(Elevator.L1, Shooter.SHOOT, GroundIntake.DOWN), true);
          put(new RobotState(Elevator.L1, Shooter.SHOOT, GroundIntake.UP), true);
          put(new RobotState(Elevator.L1, Shooter.REEF_INTAKE, GroundIntake.DOWN), true);
          // TODO: \/ ?
          put(new RobotState(Elevator.L1, Shooter.REEF_INTAKE, GroundIntake.UP), true);
          put(new RobotState(Elevator.L1, Shooter.GROUND_INTAKE, GroundIntake.DOWN), false);
          put(new RobotState(Elevator.L1, Shooter.GROUND_INTAKE, GroundIntake.UP), false);

          put(new RobotState(Elevator.BOTTOM, Shooter.STOW, GroundIntake.DOWN), true);
          put(new RobotState(Elevator.BOTTOM, Shooter.STOW, GroundIntake.UP), true); // stow
          put(new RobotState(Elevator.BOTTOM, Shooter.SHOOT, GroundIntake.DOWN), false); // TODO: ?
          put(new RobotState(Elevator.BOTTOM, Shooter.SHOOT, GroundIntake.UP), false);
          put(new RobotState(Elevator.BOTTOM, Shooter.REEF_INTAKE, GroundIntake.DOWN), false);
          put(new RobotState(Elevator.BOTTOM, Shooter.REEF_INTAKE, GroundIntake.UP), false);
          put(new RobotState(Elevator.BOTTOM, Shooter.GROUND_INTAKE, GroundIntake.DOWN), false);
          put(new RobotState(Elevator.BOTTOM, Shooter.GROUND_INTAKE, GroundIntake.UP), false);
        }
      };

  static {
    assert Elevator.values().length * Shooter.values().length * GroundIntake.values().length
        == statesValid.size();
    for (RobotState state : statesValid.keySet()) {
      Boolean valid = statesValid.get(state);
      if (valid == null || !valid.booleanValue()) continue;
      if (Validator.positionsValid(
          state.elevator.height, state.shooter.angle, state.groundIntake == State.GroundIntake.UP))
        continue;
      System.out.println(
          "WARNING: Robot state of "
              + state.toString()
              + " is registered as valid but has positions considered invalid");
    }
  }

  /**
   * Check if a given combination of states is valid
   *
   * @param elevator
   * @param shooter
   * @param groundIntake
   * @return true if the combination is registered to be valid, otherwise false
   */
  public static boolean isValid(Elevator elevator, Shooter shooter, GroundIntake groundIntake) {
    RobotState state = new RobotState(elevator, shooter, groundIntake);
    Boolean valid = statesValid.get(state);
    if (valid == null) {
      System.out.println("WARNING: Unregistered combination of states - " + state.toString());
      return true;
    }
    return true || valid.booleanValue();
  }
}
