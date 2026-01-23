package frc.robot.subsystems.superstructure.flywheel;

import edu.wpi.first.math.util.Units;
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
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.robot.subsystems.superstructure.flywheel.FlywheelConstants.*;

public class Flywheel implements Periodic {
    private static final PIDF.Tunable velocityGainsTunable = velocityGains.tunable("Superstructure/Flywheel/Velocity");
    private static final LoggedTunableNumber shootHubManualMetersPerSec = new LoggedTunableNumber("Superstructure/Flywheel/Goal/ShootHubManualMetersPerSec", 5.0);
    private static final LoggedTunableNumber shootTowerManualMetersPerSec = new LoggedTunableNumber("Superstructure/Flywheel/Goal/ShootTowerManualMetersPerSec", 5.0);
    private static final LoggedTunableNumber passManualMetersPerSec = new LoggedTunableNumber("Superstructure/Flywheel/Goal/PassManualMetersPerSec", 5.0);
    private static final LoggedTunableNumber ejectRPM = new LoggedTunableNumber("Superstructure/Flywheel/Goal/EjectRPM", -300);

    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    private final MotorIO io = createIO();
    private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum Goal {
        IDLE(() -> 0, RequestType.VoltageVolts),
        SHOOT_AND_PASS_AUTOMATIC(() -> shootHubManualMetersPerSec.get() / flywheelRadiusMeters, RequestType.VelocityRadPerSec),
        SHOOT_HUB_MANUAL(() -> shootHubManualMetersPerSec.get() / flywheelRadiusMeters, RequestType.VelocityRadPerSec),
        SHOOT_TOWER_MANUAL(() -> shootTowerManualMetersPerSec.get() / flywheelRadiusMeters, RequestType.VelocityRadPerSec),
        PASS_MANUAL(() -> passManualMetersPerSec.get() / flywheelRadiusMeters, RequestType.VelocityRadPerSec),
        EJECT(() -> Units.rotationsPerMinuteToRadiansPerSecond(ejectRPM.get()), RequestType.VelocityRadPerSec),
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
        Logger.processInputs("Inputs/Superstructure/Flywheel", inputs);

        motorDisconnectedAlert.set(!inputs.connected);

        // Apply network inputs
        if (operatorDashboard.coastOverride.hasChanged()) {
            io.setBrakeMode(!operatorDashboard.coastOverride.get());
        }

        velocityGainsTunable.ifChanged(io::setVelocityPIDF);
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Superstructure/Flywheel/Goal", goal);
        if (DriverStation.isDisabled()) {
            io.setRequest(RequestType.VoltageVolts, 0);
        } else {
            Logger.recordOutput("Superstructure/Flywheel/RequestType", goal.type);
            double value = goal.value.getAsDouble();
            Logger.recordOutput("Superstructure/Flywheel/RequestValue", value);
            io.setRequest(goal.type, value);
        }
    }
    
    public double getPositionRad() {
        return inputs.positionRad;
    }

    @AutoLogOutput(key = "Superstructure/Flywheel/AtGoal")
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
