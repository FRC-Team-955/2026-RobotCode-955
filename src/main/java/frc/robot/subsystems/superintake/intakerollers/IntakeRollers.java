package frc.robot.subsystems.superintake.intakerollers;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.motor.MotorIO;
import frc.lib.motor.MotorIOInputsAutoLogged;
import frc.lib.motor.RequestType;
import frc.lib.network.LoggedTunableNumber;
import frc.lib.subsystem.Periodic;
import frc.robot.OperatorDashboard;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.superintake.intakerollers.IntakeRollersConstants.createIO;

public class IntakeRollers implements Periodic {
    private static final LoggedTunableNumber intakeVoltage = new LoggedTunableNumber("IntakeRollers/Goal/IntakeVoltage", 12.0);

    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    private final MotorIO io = createIO();
    private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum Goal {
        IDLE(() -> 0.0, RequestType.VoltageVolts),
        INTAKE(intakeVoltage::get, RequestType.VoltageVolts),
        ;

        /** Should be constant for every loop cycle */
        private final DoubleSupplier value;
        private final RequestType type;
    }

    @Setter
    @Getter
    private Goal goal = Goal.IDLE;

    private final Alert motorDisconnectedAlert = new Alert("IntakeRollers motor is disconnected.", Alert.AlertType.kError);

    private static IntakeRollers instance;

    public static IntakeRollers get() {
        if (instance == null)
            synchronized (IntakeRollers.class) {
                instance = new IntakeRollers();
            }

        return instance;
    }

    private IntakeRollers() {
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/IntakeRollers", inputs);

        motorDisconnectedAlert.set(!inputs.connected);

        // Apply network inputs
        if (operatorDashboard.coastOverride.hasChanged()) {
            io.setBrakeMode(!operatorDashboard.coastOverride.get());
        }
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("IntakeRollers/Goal", goal);
        if (DriverStation.isDisabled()) {
            io.setRequest(RequestType.VoltageVolts, 0);
        } else {
            Logger.recordOutput("IntakeRollers/RequestType", goal.type);
            double value = goal.value.getAsDouble();
            Logger.recordOutput("IntakeRollers/RequestValue", value);
            io.setRequest(goal.type, value);
        }
    }

    /**
     * Intended to be plugged into component rotation in RobotMechanism
     */
    public double getPositionRad() {
        return inputs.positionRad;
    }
}
