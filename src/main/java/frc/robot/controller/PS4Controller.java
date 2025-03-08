package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PS4Controller implements Controller {
  private final CommandPS4Controller controller;

  public PS4Controller(int port) {
    controller = new CommandPS4Controller(port);
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
    return controller.triangle();
  }

  @Override
  public Trigger b() {
    return controller.circle();
  }

  @Override
  public Trigger back() {
    return controller.share();
  }

  @Override
  public Trigger leftBumper() {
    return controller.L1().or(controller.L2());
  }

  @Override
  public Trigger rightBumper() {
    return controller.R1().or(controller.R2());
  }

  @Override
  public Trigger start() {
    return controller.PS();
  }

  @Override
  public Trigger x() {
    return controller.square();
  }

  @Override
  public Trigger y() {
    return controller.cross(); // bruh
  }

  @Override
  public Trigger leftTrigger() {
    return controller.L1();
  }

  @Override
  public Trigger rightTrigger() {
    return controller.R1();
  }
}
