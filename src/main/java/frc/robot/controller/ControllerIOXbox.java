package frc.robot.controller;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class ControllerIOXbox extends ControllerIO {
    private final CommandXboxController controller;

    @Override
    public boolean isConnected() {
        return controller.isConnected();
    }

    @Override
    public void setRumble(double value) {
        controller.setRumble(GenericHID.RumbleType.kBothRumble, value);
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
    public Trigger a() {
        return controller.a();
    }

    @Override
    public Trigger b() {
        return controller.b();
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
    public Trigger leftBumper() {
        return controller.leftBumper();
    }

    @Override
    public Trigger rightTrigger() {
        return controller.rightTrigger();
    }

    @Override
    public Trigger rightBumper() {
        return controller.rightBumper();
    }
}
