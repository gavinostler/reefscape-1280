package frc.robot.controller;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AgnosticController implements Controller {
  static class NoOpController implements Controller {}

  private final Notifier m_notifier;
  private String name;
  private Controller controller;
  private final int port;

  public AgnosticController(int port) {
    this.port = port;
    m_notifier = new Notifier(this::refresh);
    refresh();
    m_notifier.startPeriodic(1);
  }

  public void refresh() {
    String name = DriverStation.getJoystickName(port).toLowerCase();
    if (name.equals(this.name)) return;
    this.name = name;
    controller = findController(name, port);
  }

  private static Controller findController(String name, int port) {
    if (name.contains("xbox")) {
      return new XboxController(port);
    }
    if (name.contains("ps4")) {
      return new PS4Controller(port);
    }
    System.out.println("NOT XBOX !!! ");
    return new NoOpController();
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
    return controller.resetHeading();
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
