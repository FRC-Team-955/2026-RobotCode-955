package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public abstract class ControllerIO {
    public abstract boolean isConnected();

    public abstract void setRumble(double value);

    public abstract double getLeftX();

    public abstract double getLeftY();

    public abstract double getRightX();

    public abstract double getRightY();

    public abstract Trigger a();

    public abstract Trigger b();

    public abstract Trigger x();

    public abstract Trigger y();

    public abstract Trigger leftTrigger();

    public abstract Trigger leftBumper();

    public abstract Trigger rightTrigger();

    public abstract Trigger rightBumper();
}
