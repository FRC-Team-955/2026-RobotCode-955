package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.motor.MotorIO;
import frc.lib.motor.MotorIOInputsAutoLogged;
import frc.lib.motor.RequestType;
import frc.lib.subsystem.Periodic;
import frc.robot.OperatorDashboard;
import frc.robot.RobotMechanism;
import frc.robot.subsystems.elevator.Elevator;
import lombok.Getter;
import lombok.NonNull;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.endeffector.EndEffectorConstants.*;
import static frc.robot.subsystems.endeffector.EndEffectorTuning.*;

public class EndEffector implements Periodic {
    private final RobotMechanism robotMechanism = RobotMechanism.get();
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private final Elevator elevator = Elevator.get();

    private final MotorIO io = createIO();
    private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum Goal {
        IDLE(() -> 0, RequestType.VoltageVolts),
        FUNNEL_INTAKE(funnelIntakeGoalSetpoint::get, RequestType.VelocityRadPerSec),
        SCORE_CORAL(scoreCoralGoalSetpoint::get, RequestType.VelocityRadPerSec),
        SCORE_CORAL_L1(scoreCoralL1GoalSetpoint::get, RequestType.VelocityRadPerSec),
        DESCORE_ALGAE(descoreAlgaeGoalSetpoint::get, RequestType.VelocityRadPerSec),
        EJECT_ALTERNATE(() -> Timer.getTimestamp() % 1.0 < 0.86 ? ejectGoalSetpoint.get() : -ejectGoalSetpoint.get(), RequestType.VelocityRadPerSec),
        ZERO_CORAL(zeroCoralGoalSetpoint::get, RequestType.VelocityRadPerSec),
        HOME_INITIAL(() -> relativePositionOriginRad + rollersRadiansForMeters(homeInitialMeters.get()), RequestType.PositionRad),
        HOME_FINAL(() -> relativePositionOriginRad + rollersRadiansForMeters(homeFinalMeters.get()), RequestType.PositionRad),
        ;

        /** Should be constant for every loop cycle */
        private final DoubleSupplier value;
        private final RequestType type;
    }

    @Getter
    private Goal goal = Goal.IDLE;

    @AutoLogOutput(key = "EndEffector/RelativePositionOriginRad")
    private static double relativePositionOriginRad = 0.0;

    public void setGoal(@NonNull Goal goal) {
        this.goal = goal;
        if (goal == Goal.HOME_INITIAL || goal == Goal.HOME_FINAL) {
            relativePositionOriginRad = inputs.positionRad;
        }
    }

    private final Alert motorDisconnectedAlert = new Alert("End effector motor is disconnected.", Alert.AlertType.kError);

    private static EndEffector instance;

    public static EndEffector get() {
        if (instance == null)
            synchronized (EndEffector.class) {
                instance = new EndEffector();
            }

        return instance;
    }

    private EndEffector() {
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/EndEffector", inputs);

        motorDisconnectedAlert.set(!inputs.connected);

        // Update mechanism

        // Apply network inputs
        if (operatorDashboard.coastOverride.hasChanged()) {
            io.setBrakeMode(!operatorDashboard.coastOverride.get());
        }

        positionGainsTunable.ifChanged(io::setPositionPIDF);
        velocityGainsTunable.ifChanged(io::setVelocityPIDF);
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("EndEffector/Goal", goal);
        if (DriverStation.isDisabled()) {
            io.setRequest(RequestType.VoltageVolts, 0);
        } else {
            Logger.recordOutput("EndEffector/RequestType", goal.type);
            double value = goal.value.getAsDouble();
            Logger.recordOutput("EndEffector/RequestValue", value);
            io.setRequest(goal.type, value);
        }
    }

    @AutoLogOutput(key = "EndEffector/AtGoal")
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

    @AutoLogOutput(key = "EndEffector/DescoreAlgaeAmperageTriggered")
    private boolean descoreAlgaeAmperageTriggered() {
        return Math.abs(inputs.currentAmps) > descoreAlgaeTriggerAmps;
    }

    public Command waitUntilDescoreAlgaeAmperageTriggered() {
        return Commands.waitUntil(this::descoreAlgaeAmperageTriggered);
    }

    @AutoLogOutput(key = "EndEffector/AngleRad")
    public double getAngleRad() {
        return MathUtil.interpolate(
                angleWhenRetractedRad,
                angleWhenExtendedRad,
                (elevator.getPositionMeters() - extendStartMeters) / extendDistanceMeters
        );
    }
}