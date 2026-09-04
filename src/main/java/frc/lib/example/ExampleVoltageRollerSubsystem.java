package frc.lib.example;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.Util;
import frc.lib.devices.motor.CtrlSparkMaxConfig;
import frc.lib.devices.motor.MechanismSim;
import frc.lib.devices.motor.Motor;
import frc.lib.network.LoggedTunableNumber;
import frc.lib.subsystem.Periodic;
import frc.robot.BuildConstants;
import frc.robot.OperatorDashboard;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class ExampleVoltageRollerSubsystem implements Periodic {
    private static final LoggedTunableNumber rollVoltage = new LoggedTunableNumber("ExampleVoltageRollerSubsystem/Goal/RollVoltage", 3.0);

    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    private final Motor motor = Motor
            .createSparkMax(
                    "ExampleVoltageRollerSubsystem",
                    -1,
                    new CtrlSparkMaxConfig()
                            .withCurrentLimit(40)
                            .withInverted(false)
                            .withGearRatio(5)
                            .withNeutralMode(NeutralModeValue.Coast),
                    0.0,
                    MechanismSim.roller(0.01)
            );

    @RequiredArgsConstructor
    public enum Goal {
        IDLE(() -> 0),
        ROLL(rollVoltage::get),
        ;

        /** Should be constant for every loop cycle */
        private final DoubleSupplier voltageVolts;
    }

    @Setter
    @Getter
    private Goal goal = Goal.IDLE;

    private static ExampleVoltageRollerSubsystem instance;

    public static synchronized ExampleVoltageRollerSubsystem get() {
        if (instance == null) {
            instance = new ExampleVoltageRollerSubsystem();
        }

        return instance;
    }

    private ExampleVoltageRollerSubsystem() {
        if (instance != null) {
            Util.error("Duplicate ExampleVoltageRollerSubsystem created");
        }
    }

    @Override
    public void periodicBeforeCommands() {
        // Apply network inputs
        if (!motor.isEmergencyStopped() && operatorDashboard.coastOverride.hasChanged()) {
            motor.setNeutralMode(operatorDashboard.coastOverride.get() ? NeutralModeValue.Coast : NeutralModeValue.Brake);
        }
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("ExampleVoltageRollerSubsystem/Goal", goal);
        if (DriverStation.isDisabled()) {
            motor.setVoltageRequest(0);
        } else {
            double volts = goal.voltageVolts.getAsDouble();
            motor.setVoltageRequest(volts);
            if (BuildConstants.isSimOrReplay) {
                Logger.recordOutput("ExampleVoltageRollerSubsystem/RequestedVolts", volts);
            }
        }
    }
}
