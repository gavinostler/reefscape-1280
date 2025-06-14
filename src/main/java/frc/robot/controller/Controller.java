package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface Controller {
  static Trigger no_op = new Trigger(() -> false);

  default double getLeftX() {
    return 0.0;
  }

  default double getLeftY() {
    return 0.0;
  }

  default double getRightX() {
    return 0.0;
  }

  default double getRightY() {
    return 0.0;
  }

  default Trigger resetHeading() {
    return no_op;
  }

  default Trigger povDown() {
    return no_op;
  }

  default Trigger povUp() {
    return no_op;
  }

  default Trigger povLeft() {
    return no_op;
  }

  default Trigger povRight() {
    return no_op;
  }

  default Trigger leftBumper() {
    return no_op;
  }

  default Trigger rightBumper() {
    return no_op;
  }

  default Trigger start() {
    return no_op;
  }

  default Trigger back() {
    return no_op;
  }

  default Trigger x() {
    return no_op;
  }

  default Trigger y() {
    return no_op;
  }

  default Trigger a() {
    return no_op;
  }

  default Trigger b() {
    return no_op;
  }

  default Trigger rightTrigger() {
    return no_op;
  }

  default Trigger leftTrigger() {
    return no_op;
  }

  default Trigger rightStick() {
    return no_op;
  }
}
