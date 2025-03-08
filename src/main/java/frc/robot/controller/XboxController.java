package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxController implements Controller {
  private final CommandXboxController controller;

  public XboxController(int port) {
    controller = new CommandXboxController(port);
  }

  @Override
  public double getLeftX() {
    return controller.getLeftX();
  }

  @Override
  public double getLeftY() {
    return controller.getLeftY();
  }

  @Override
  public double getRightX() {
    return controller.getRightX();
  }

  @Override
  public double getRightY() {
    return controller.getRightY();
  }

  @Override
  public Trigger resetHeading() {
    return controller.leftStick();
  }

  @Override
  public Trigger povDown() {
    return controller.povDown();
  }

  @Override
  public Trigger povLeft() {
    return controller.povLeft();
  }

  @Override
  public Trigger povRight() {
    return controller.povRight();
  }

  @Override
  public Trigger povUp() {
    return controller.povUp();
  }

  @Override
  public Trigger a() {
    return controller.a();
  }

  @Override
  public Trigger b() {
    return controller.b();
  }

  @Override
  public Trigger back() {
    return controller.back();
  }

  @Override
  public Trigger leftBumper() {
    return controller.leftBumper();
  }

  @Override
  public Trigger rightBumper() {
    return controller.rightBumper();
  }

  @Override
  public Trigger start() {
    return controller.start();
  }

  @Override
  public Trigger x() {
    return controller.x();
  }

  @Override
  public Trigger y() {
    return controller.y();
  }

  @Override
  public Trigger leftTrigger() {
    return controller.leftTrigger();
  }

  @Override
  public Trigger rightTrigger() {
    return controller.rightTrigger();
  }

  @Override
  public Trigger rightStick() {
    return controller.rightStick();
  }
}
