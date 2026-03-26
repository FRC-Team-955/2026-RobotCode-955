package frc.robot.subsystems.superintake.intakerollers;

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

import static frc.robot.subsystems.superintake.intakerollers.IntakeRollersConstants.createIO;

public class IntakeRollers implements Periodic {
    private static final LoggedTunableNumber idleVoltage = new LoggedTunableNumber("Superintake/IntakeRollers/Goal/IdleVoltage", 0.0);
    private static final LoggedTunableNumber intakeVoltage = new LoggedTunableNumber("Superintake/IntakeRollers/Goal/IntakeVoltage", 12.0);
    private static final LoggedTunableNumber ejectVoltage = new LoggedTunableNumber("Superintake/IntakeRollers/Goal/EjectVoltage", -12.0);
    private static final EnergyLogger energyLogger = EnergyLogger.get();

    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    private final MotorIO io = createIO();
    private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();


    @RequiredArgsConstructor
    public enum Goal {
        IDLE(idleVoltage::get, RequestType.VoltageVolts),
        INTAKE(intakeVoltage::get, RequestType.VoltageVolts),
        EJECT(ejectVoltage::get, RequestType.VoltageVolts),
        ;

        /** Should be constant for every loop cycle */
        private final DoubleSupplier value;
        private final RequestType type;
    }

    @Setter
    @Getter
    private Goal goal = Goal.IDLE;

    private final Alert motorDisconnectedAlert = new Alert("Intake rollers motor is disconnected.", Alert.AlertType.kError);
    public final Alert highTemperatureAlert = new Alert("Intake rollers motor temperature is high.", Alert.AlertType.kWarning);

    private static IntakeRollers instance;

    public static synchronized IntakeRollers get() {
        if (instance == null) {
            instance = new IntakeRollers();
        }

        return instance;
    }

    private IntakeRollers() {
        if (instance != null) {
            Util.error("Duplicate IntakeRollers created");
        }
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Superintake/IntakeRollers", inputs);

        motorDisconnectedAlert.set(!inputs.connected);
        highTemperatureAlert.set(inputs.temperatureCelsius > 50);

        energyLogger.reportCurrentUsage("IntakeRollers", inputs.connected ? inputs.supplyCurrentAmps : 0.0);
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Superintake/IntakeRollers/Goal", goal);
        if (DriverStation.isDisabled()) {
            io.setRequest(RequestType.VoltageVolts, 0);
        } else {
            double value = goal.value.getAsDouble();
            io.setRequest(goal.type, value);
            if (BuildConstants.isSimOrReplay) {
                Logger.recordOutput("Superintake/IntakeRollers/RequestType", goal.type);
                Logger.recordOutput("Superintake/IntakeRollers/RequestValue", value);
            }
        }
    }

    public double getPositionRad() {
        return inputs.positionRad;
    }

    public Transform3d intakeRollersTransform() {
        return new Transform3d(
                new Translation3d(Units.inchesToMeters(19.75), 0.0, Units.inchesToMeters(8.985680)),
                new Rotation3d()
        ).plus(new Transform3d(
                new Translation3d(),
                new Rotation3d(0.0, getPositionRad(), 0.0)
        ));
    }
}
