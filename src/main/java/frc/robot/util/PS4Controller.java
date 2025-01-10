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
}
