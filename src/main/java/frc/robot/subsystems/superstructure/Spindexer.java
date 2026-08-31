package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.EnergyLogger;
import frc.lib.Util;
import frc.lib.devices.motor.MechanismSim;
import frc.lib.devices.motor.Motor;
import frc.lib.network.LoggedTunableNumber;
import frc.lib.subsystem.Periodic;
import frc.robot.BuildConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class Spindexer implements Periodic {
    private static final LoggedTunableNumber feedVoltage = new LoggedTunableNumber("Superstructure/Spindexer/Goal/FeedVoltage", 12.0);
    private static final LoggedTunableNumber ejectVoltage = new LoggedTunableNumber("Superstructure/Spindexer/Goal/EjectVoltage", -12.0);

    private static final EnergyLogger energyLogger = EnergyLogger.get();

    private final Motor motor = Motor.createTalonFX(
            "Superstructure/Spindexer",
            13,
            new TalonFXConfiguration()
                    .withMotorOutput(new MotorOutputConfigs()
                            .withNeutralMode(NeutralModeValue.Coast)
                            .withInverted(InvertedValue.Clockwise_Positive))
                    .withCurrentLimits(new CurrentLimitsConfigs()
                            .withStatorCurrentLimit(90)
                            .withSupplyCurrentLimit(50))
                    .withFeedback(new FeedbackConfigs()
                            .withSensorToMechanismRatio(5)),
            0.0,
            MechanismSim.roller(0.01)
    );

    @RequiredArgsConstructor
    public enum Goal {
        IDLE(() -> 0),
        FEED(() -> /*Timer.getTimestamp() % 2.0 < 0.1 ? -feedVoltage.get() :*/ feedVoltage.get()),
        EJECT(ejectVoltage::get),
        EJECT_ALTERNATE(() -> Timer.getTimestamp() % 0.5 < 0.25 ? ejectVoltage.get() : -ejectVoltage.get()),
        AGITATE(() -> Timer.getTimestamp() % 3.0 < 1.0 ? Math.sin(2.0 * Math.PI * Timer.getTimestamp()) : 0.0),
        ;

        /** Should be constant for every loop cycle */
        private final DoubleSupplier voltageVolts;
    }

    @Setter
    @Getter
    private Goal goal = Goal.IDLE;

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
    public void periodicAfterCommands() {
        Logger.recordOutput("Superstructure/Spindexer/Goal", goal);
        if (DriverStation.isDisabled()) {
            motor.setVoltageRequest(0);
        } else {
            double voltageVolts = goal.voltageVolts.getAsDouble();
            motor.setVoltageRequest(voltageVolts);
            if (BuildConstants.isSimOrReplay) {
                Logger.recordOutput("Superstructure/Spindexer/RequestedVolts", voltageVolts);
            }
        }
    }

    public Transform3d getMechanismTransform() {
        return new Transform3d(
                new Translation3d(0.0, Units.inchesToMeters(1.4), Units.inchesToMeters(12.0)),
                new Rotation3d(0.0, 0.0, Units.degreesToRadians(90.0))
        ).plus(new Transform3d(
                new Translation3d(),
                new Rotation3d(0.0, motor.getPositionRad(), 0.0)
        ));
    }
}
