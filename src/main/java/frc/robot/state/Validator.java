package frc.robot.state;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Validator implements Sendable {
  public ElevatorSubsystem elevator;
  public ShooterSubsystem shooter;
  public GroundIntakeSubsystem groundIntake;

  // Set to false to consider both state and position as always valid
  public static boolean safetyEnabled = false;

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

  // View in Glass
  private class Display {
    Mechanism2d mech = new Mechanism2d(6, 10, new Color8Bit(0, 0, 0));
    MechanismRoot2d root = mech.getRoot("robot", 3, 1);

    // TODO: real vs state ground intake
    MechanismLigament2d groundIntake;
    MechanismLigament2d stateElevator;
    MechanismLigament2d realElevator;
    MechanismLigament2d stateShooter;
    MechanismLigament2d realShooter;

    static final double GROUND_INTAKE_ANGLE_UP = 100;
    static final double GROUND_INTAKE_ANGLE_DOWN = 10;
    static final int ELEVATOR_HEIGHT_MIN = 2;
    static final int ELEVATOR_HEIGHT_MAX = 7;

    void update() {
      groundIntake.setAngle(
          switch (Validator.this.groundIntake.getState()) {
            case DOWN -> GROUND_INTAKE_ANGLE_DOWN;
            case UP -> GROUND_INTAKE_ANGLE_UP;
          });
      stateElevator.setLength( // Elevator state preview
          ELEVATOR_HEIGHT_MIN
              + (ELEVATOR_HEIGHT_MAX - ELEVATOR_HEIGHT_MIN)
                  * Validator.this.elevator.getState().height);
      realElevator.setLength( // Actual height from encoder
          ELEVATOR_HEIGHT_MIN
              + (ELEVATOR_HEIGHT_MAX - ELEVATOR_HEIGHT_MIN) * Validator.this.elevator.getHeight());
      stateShooter.setAngle( // Shooter state preview
          Validator.this.shooter.getState().angle * 360.0);
      realShooter.setAngle( // Actual height from encoder
          Validator.this.shooter.getArmAngle() * 360.0);
    }

    public Display() { // Ignore horrid API
      root.append(new MechanismLigament2d("drivebaseLeft", 2, 0));
      groundIntake =
          root.append(new MechanismLigament2d("drivebaseRight", 2, 0))
              .append(new MechanismLigament2d("groundIntake", 3, GROUND_INTAKE_ANGLE_UP));
      stateElevator =
          root.append(
              new MechanismLigament2d(
                  "stateElevator", ELEVATOR_HEIGHT_MIN, 90, 6, new Color8Bit(255, 0, 255)));
      realElevator = root.append(new MechanismLigament2d("realElevator", ELEVATOR_HEIGHT_MIN, 90));
      stateShooter =
          stateElevator.append(
              new MechanismLigament2d("stateShooter", 2, 0, 6, new Color8Bit(255, 0, 255)));
      realShooter = realElevator.append(new MechanismLigament2d("realShooter", 2, 0));

      update();
    }
  }
}
