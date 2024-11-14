package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class XboxController implements Controller {
    private final CommandXboxController controller;
    public XboxController() {
        this.controller = new CommandXboxController(Constants.Operator.kDriverControllerPort);
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
    public Trigger resetHeading() {
        return this.controller.leftBumper();
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
