package frc.robot.subsystems.superstructure.spindexer;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.EnergyLogger;
import frc.lib.Util;
import frc.lib.motor.MotorIO;
import frc.lib.motor.MotorIOInputsAutoLogged;
import frc.lib.motor.RequestType;
import frc.lib.network.LoggedTunableNumber;
import frc.lib.subsystem.Periodic;
import frc.robot.BuildConstants;
import frc.robot.OperatorDashboard;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.superstructure.spindexer.SpindexerConstants.createIO;

public class Spindexer implements Periodic {
    private static final LoggedTunableNumber feedVoltage = new LoggedTunableNumber("Superstructure/Spindexer/Goal/FeedVoltage", 12.0);
    private static final LoggedTunableNumber ejectVoltage = new LoggedTunableNumber("Superstructure/Spindexer/Goal/EjectVoltage", -12.0);

    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private static final EnergyLogger energyLogger = EnergyLogger.get();

    private final MotorIO io = createIO();
    private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum Goal {
        IDLE(() -> 0, RequestType.VoltageVolts),
        FEED(feedVoltage::get, RequestType.VoltageVolts),
        EJECT(ejectVoltage::get, RequestType.VoltageVolts),
        EJECT_ALTERNATE(() -> Timer.getTimestamp() % 0.5 < 0.25 ? ejectVoltage.get() : -ejectVoltage.get(), RequestType.VoltageVolts),
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

        energyLogger.reportCurrentUsage("Spindexer", inputs.connected ? inputs.supplyCurrentAmps : 0.0);
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Superstructure/Spindexer/Goal", goal);
        if (DriverStation.isDisabled()) {
            io.setRequest(RequestType.VoltageVolts, 0);
        } else {
            double value = goal.value.getAsDouble();
            io.setRequest(goal.type, value);
            if (BuildConstants.isSimOrReplay) {
                Logger.recordOutput("Superstructure/Spindexer/RequestType", goal.type);
                Logger.recordOutput("Superstructure/Spindexer/RequestValue", value);
            }
        }
    }

    public double getPositionRad() {
        return inputs.positionRad;
    }

    public Transform3d spindexerTransform() {
        return new Transform3d(
                new Translation3d(0.0, Units.inchesToMeters(1.4), Units.inchesToMeters(12.0)),
                new Rotation3d(0.0, 0.0, Units.degreesToRadians(90.0))
        ).plus(new Transform3d(
                new Translation3d(),
                new Rotation3d(0.0, getPositionRad(), 0.0)
        ));
    }
}
