package frc.robot.subsystems.superstructure.flywheel;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.PIDF;
import frc.lib.motor.MotorIO;
import frc.lib.motor.MotorIOInputsAutoLogged;
import frc.lib.motor.RequestType;
import frc.lib.network.LoggedTunableNumber;
import frc.lib.subsystem.Periodic;
import frc.robot.OperatorDashboard;
import frc.robot.subsystems.superstructure.Superstructure;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.robot.subsystems.superstructure.flywheel.FlywheelConstants.*;

public class Flywheel implements Periodic {
    private static final PIDF.Tunable velocityGainsTunable = velocityGains.tunable("Flywheel/Velocity");
    private static final LoggedTunableNumber runAtVoltage = new LoggedTunableNumber("Flywheel/Goal/RunAtVoltage", 3.0);
    private static final LoggedTunableNumber runAtSpeed = new LoggedTunableNumber("Flywheel/Goal/RunAtSpeed", 20.0);

    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    private final MotorIO io = createIO();
    private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum Goal {
        IDLE(() -> 0, RequestType.VoltageVolts),
        RUN_AT_VOLTAGE(runAtVoltage::get, RequestType.VoltageVolts),
        RUN_AT_CERTAIN_SPEED(runAtSpeed::get, RequestType.VelocityRadPerSec)
        ;

        /** Should be constant for every loop cycle */
        private final DoubleSupplier value;
        private final RequestType type;
    }

    @Getter
    private Goal goal = Goal.IDLE;

    private final Alert motorDisconnectedAlert = new Alert("Flywheel motor is disconnected.", Alert.AlertType.kError);

    private static Flywheel instance;

    public static Flywheel get() {
        if (instance == null)
            synchronized (Flywheel.class) {
                instance = new Flywheel();
            }

        return instance;
    }

    private Flywheel() {
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Flywheel", inputs);

        motorDisconnectedAlert.set(!inputs.connected);

        // Apply network inputs
        if (operatorDashboard.coastOverride.hasChanged()) {
            io.setBrakeMode(!operatorDashboard.coastOverride.get());
        }

        velocityGainsTunable.ifChanged(io::setVelocityPIDF);
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Flywheel/Goal", goal);
        if (DriverStation.isDisabled()) {
            io.setRequest(RequestType.VoltageVolts, 0);
        } else {
            Logger.recordOutput("Flywheel/RequestType", goal.type);
            double value = goal.value.getAsDouble();
            Logger.recordOutput("Flywheel/RequestValue", value);
            io.setRequest(goal.type, value);
        }
    }

    /**
     * Intended to be plugged into component rotation in RobotMechanism
     */
    public double getPositionRad() {
        return inputs.positionRad;
    }

    @AutoLogOutput(key = "Flywheel/AtGoal")
    public boolean atGoal() {
        double value = goal.value.getAsDouble();
        return switch (goal.type) {
            case PositionRad -> Math.abs(inputs.positionRad - value) <= tolerances.positionToleranceRad();

            case VelocityRadPerSec ->
                    Math.abs(inputs.velocityRadPerSec - value) <= tolerances.velocityToleranceRadPerSec();

            case VoltageVolts -> Math.abs(inputs.appliedVolts - value) <= tolerances.voltageToleranceVolts();
        };
    }

    public Command waitUntilAtGoal() {
        return Commands.waitUntil(this::atGoal);
    }

    public Command setGoal(Goal flywheelGoal) {
        return runOnce(() -> {
            goal = flywheelGoal;
        });
    }
}
