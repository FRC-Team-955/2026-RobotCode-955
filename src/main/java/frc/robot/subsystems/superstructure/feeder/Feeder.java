package frc.robot.subsystems.superstructure.feeder;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
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

import static frc.robot.subsystems.superstructure.feeder.FeederConstants.createIO;

public class Feeder implements Periodic {
    private static final LoggedTunableNumber feedVoltage = new LoggedTunableNumber("Superstructure/Feeder/Goal/FeedVoltage", 12.0);
    private static final LoggedTunableNumber ejectVoltage = new LoggedTunableNumber("Superstructure/Feeder/Goal/EjectVoltage", -12.0);

    private static final EnergyLogger energyLogger = EnergyLogger.get();
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

    private final Alert motorDisconnectedAlert = new Alert("Feeder motor is disconnected.", Alert.AlertType.kError);

    private static Feeder instance;

    public static synchronized Feeder get() {
        if (instance == null) {
            instance = new Feeder();
        }

        return instance;
    }

    private Feeder() {
        if (instance != null) {
            Util.error("Duplicate Feeder created");
        }
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Superstructure/Feeder", inputs);

        motorDisconnectedAlert.set(!inputs.connected);

        energyLogger.reportPowerUsage("Feeder", inputs.connected ? inputs.appliedVolts * inputs.supplyCurrentAmps : 0.0);

        // Apply network inputs
        if (operatorDashboard.coastOverride.hasChanged()) {
            io.setNeutralMode(operatorDashboard.coastOverride.get() ? NeutralModeValue.Coast : NeutralModeValue.Brake);
        }
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Superstructure/Feeder/Goal", goal);
        if (DriverStation.isDisabled()) {
            io.setRequest(RequestType.VoltageVolts, 0);
        } else {
            double value = goal.value.getAsDouble();
            io.setRequest(goal.type, value);
            if (BuildConstants.isSimOrReplay) {
                Logger.recordOutput("Superstructure/Feeder/RequestType", goal.type);
                Logger.recordOutput("Superstructure/Feeder/RequestValue", value);
            }
        }
    }

    public double getPositionRad() {
        return inputs.positionRad;
    }

    public boolean feederDisconnected() {
        return !inputs.connected;
    }


    public Transform3d feederTransform() {
        return new Transform3d(
                new Translation3d(Units.inchesToMeters(-3.451296), Units.inchesToMeters(-5.445256), Units.inchesToMeters(8.430151)),
                new Rotation3d(0.0, 0.0, Units.degreesToRadians(90.0))
        ).plus((new Transform3d(
                new Translation3d(),
                new Rotation3d(0.0, getPositionRad(), 0.0)
        )));
    }
}
