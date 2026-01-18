package frc.lib.example;

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
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.lib.example.ExampleRollerSubsystemConstants.*;

public class ExampleRollerSubsystem implements Periodic {
    private static final PIDF.Tunable positionGainsTunable = positionGains.tunable("ExampleRollerSubsystem/Position");
    private static final PIDF.Tunable velocityGainsTunable = velocityGains.tunable("ExampleRollerSubsystem/Velocity");
    private static final LoggedTunableNumber runAtVoltage = new LoggedTunableNumber("ExampleRollerSubsystem/Goal/RunAtVoltage", 3.0);

    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    private final MotorIO io = createIO();
    private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum Goal {
        IDLE(() -> 0, RequestType.VoltageVolts),
        RUN_AT_VOLTAGE(runAtVoltage::get, RequestType.VoltageVolts),
        RUN_AT_CERTAIN_SPEED(() -> 20, RequestType.VelocityRadPerSec),
        RUN_TO_CERTAIN_POSITION(() -> 3, RequestType.PositionRad),
        ;

        /** Should be constant for every loop cycle */
        private final DoubleSupplier value;
        private final RequestType type;
    }

    @Setter
    @Getter
    private Goal goal = Goal.IDLE;

    private final Alert motorDisconnectedAlert = new Alert("ExampleRollerSubsystem motor is disconnected.", Alert.AlertType.kError);

    private static ExampleRollerSubsystem instance;

    public static ExampleRollerSubsystem get() {
        if (instance == null)
            synchronized (ExampleRollerSubsystem.class) {
                instance = new ExampleRollerSubsystem();
            }

        return instance;
    }

    private ExampleRollerSubsystem() {
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/ExampleRollerSubsystem", inputs);

        motorDisconnectedAlert.set(!inputs.connected);

        // Apply network inputs
        if (operatorDashboard.coastOverride.hasChanged()) {
            io.setBrakeMode(!operatorDashboard.coastOverride.get());
        }

        positionGainsTunable.ifChanged(io::setPositionPIDF);
        velocityGainsTunable.ifChanged(io::setVelocityPIDF);
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("ExampleRollerSubsystem/Goal", goal);
        if (DriverStation.isDisabled()) {
            io.setRequest(RequestType.VoltageVolts, 0);
        } else {
            Logger.recordOutput("ExampleRollerSubsystem/RequestType", goal.type);
            double value = goal.value.getAsDouble();
            Logger.recordOutput("ExampleRollerSubsystem/RequestValue", value);
            io.setRequest(goal.type, value);
        }
    }

    /**
     * Intended to be plugged into component rotation in RobotMechanism
     */
    public double getPositionRad() {
        return inputs.positionRad;
    }

    @AutoLogOutput(key = "ExampleRollerSubsystem/AtGoal")
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
}
