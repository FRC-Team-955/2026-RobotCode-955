package frc.robot.subsystems.superstructure.spindexer;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.Util;
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

import static frc.robot.subsystems.superstructure.spindexer.SpindexerConstants.createIO;

public class Spindexer implements Periodic {
    private static final LoggedTunableNumber feedVoltage = new LoggedTunableNumber("Superstructure/Spindexer/Goal/FeedVoltage", 6.7);
    private static final LoggedTunableNumber ejectVoltage = new LoggedTunableNumber("Superstructure/Spindexer/Goal/EjectVoltage", -3.0);

    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    private final MotorIO io = createIO();
    private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum Goal {
        IDLE(() -> 0, RequestType.VoltageVolts),
        FEED(feedVoltage::get, RequestType.VoltageVolts),
        EJECT(ejectVoltage::get, RequestType.VoltageVolts),
        ;

        /** Should be constant for every loop cycle */
        private final DoubleSupplier value;
        private final RequestType type;
    }

    @Setter
    @Getter
    private Goal goal = Goal.IDLE;

    private final Alert motorDisconnectedAlert = new Alert("Spindexer motor is disconnected.", Alert.AlertType.kError);

    private static Spindexer instance;

    public static synchronized Spindexer get() {
        if (instance == null) {
            instance = new Spindexer();
        }

        return instance;
    }

    private Spindexer() {
        if (instance != null) {
            Util.error("Duplicate Spindexer created");
        }
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Superstructure/Spindexer", inputs);

        motorDisconnectedAlert.set(!inputs.connected);
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Superstructure/Spindexer/Goal", goal);
        if (DriverStation.isDisabled()) {
            io.setRequest(RequestType.VoltageVolts, 0);
        } else {
            Logger.recordOutput("Superstructure/Spindexer/RequestType", goal.type);
            double value = goal.value.getAsDouble();
            Logger.recordOutput("Superstructure/Spindexer/RequestValue", value);
            io.setRequest(goal.type, value);
        }
    }

    public double getPositionRad() {
        return inputs.positionRad;
    }
}
