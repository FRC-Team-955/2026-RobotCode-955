package frc.robot.controller;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class ControllerIOPS5 extends ControllerIO {
    private final CommandPS5Controller controller;

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
        return controller.cross();
    }

    @Override
    public Trigger b() {
        return controller.circle();
    }

    @Override
    public Trigger x() {
        return controller.square();
    }

    @Override
    public Trigger y() {
        return controller.triangle();
    }

    @Override
    public Trigger leftTrigger() {
        return controller.L2();
    }

    @Override
    public Trigger leftBumper() {
        return controller.L1();
    }

    @Override
    public Trigger rightTrigger() {
        return controller.R2();
    }

    @Override
    public Trigger rightBumper() {
        return controller.R1();
    }
}
