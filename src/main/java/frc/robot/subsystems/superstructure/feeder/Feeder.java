package frc.robot.subsystems.superstructure.feeder;

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

import static frc.robot.subsystems.superstructure.feeder.FeederConstants.createIO;

public class Feeder implements Periodic {
    private static final LoggedTunableNumber feedVoltage = new LoggedTunableNumber("Superstructure/Feeder/Goal/FeedVoltage", 3.0);
    private static final LoggedTunableNumber ejectVoltage = new LoggedTunableNumber("Superstructure/Feeder/Goal/EjectVoltage", -3.0);

    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();

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

    private final Alert motorDisconnectedAlert = new Alert("Feeder motor is disconnected.", Alert.AlertType.kError);

    private static Feeder instance;

    public static Feeder get() {
        if (instance == null)
            synchronized (Feeder.class) {
                instance = new Feeder();
            }

        return instance;
    }

    private Feeder() {
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Superstructure/Feeder", inputs);

        motorDisconnectedAlert.set(!inputs.connected);

        // Apply network inputs
        if (operatorDashboard.coastOverride.hasChanged()) {
            io.setBrakeMode(!operatorDashboard.coastOverride.get());
        }
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Superstructure/Feeder/Goal", goal);
        if (DriverStation.isDisabled()) {
            io.setRequest(RequestType.VoltageVolts, 0);
        } else {
            Logger.recordOutput("Superstructure/Feeder/RequestType", goal.type);
            double value = goal.value.getAsDouble();
            Logger.recordOutput("Superstructure/Feeder/RequestValue", value);
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
