package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Operator;

public class AgnosticController implements Controller {
    static class NoOpController implements Controller {}

    private final Notifier m_notifier;
    private String name;
    private Controller controller;

    public AgnosticController() {
        this.m_notifier = new Notifier(this::refresh);
        refresh();
        this.m_notifier.startPeriodic(1);
    }

    public void refresh() {
        String name = DriverStation.getJoystickName(Operator.kDriverControllerPort).toLowerCase();
        if(name.equals(this.name)) return;
        this.name = name;
        this.controller = findController(name);
    }

    private static Controller findController(String name) {
        if (name.contains("keyboard") || name.contains("xbox")) {
            return new XboxController();
        }
        if (name.contains("ps4")) {
            return new PS4Controller();
        }

        return new NoOpController();
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
        return this.controller.resetHeading();
    }
}
