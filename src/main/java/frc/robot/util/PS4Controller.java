package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class PS4Controller implements Controller {
  private final CommandPS4Controller controller;

  public PS4Controller() {
    this.controller = new CommandPS4Controller(Constants.Operator.kDriverControllerPort);
  }

  @Override
  public double getLeftX() {
    return this.controller.getLeftX();
  }

  @Override
  public double getLeftY() {
    return this.controller.getLeftY();
  }

  @Override
  public double getRightX() {
    return this.controller.getRightX();
  }

  @Override
  public double getRightY() {
    return this.controller.getRightY();
  }

  @Override
  public Trigger povDown() {
    return this.controller.povDown();
  }

  @Override
  public Trigger povLeft() {
    return this.controller.povLeft();
  }

  @Override
  public Trigger povRight() {
    return this.controller.povRight();
  }

  @Override
  public Trigger povUp() {
    return this.controller.povUp();
  }

  @Override
  public Trigger a() {
    return this.controller.triangle();
  }

  @Override
  public Trigger b() {
    return this.controller.circle();
  }

  @Override
  public Trigger back() {
    return this.controller.share();
  }

  @Override
  public Trigger leftBumper() {
    return this.controller.L1().or(this.controller.L2());
  }

  @Override
  public Trigger rightBumper() {
    return this.controller.R1().or(this.controller.R2());
  }

  @Override
  public Trigger start() {
    return this.controller.PS();
  }

  @Override
  public Trigger x() {
    return this.controller.square();
  }

  @Override
  public Trigger y() {
    return this.controller.cross(); // bruh
  }
}
