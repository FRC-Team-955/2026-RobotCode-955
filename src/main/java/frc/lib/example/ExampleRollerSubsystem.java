package frc.lib.example;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.devices.motor.CtrlSparkMaxConfig;
import frc.lib.devices.motor.MechanismSim;
import frc.lib.devices.motor.Motor;
import frc.lib.network.LoggedTunableNumber;
import frc.lib.network.LoggedTunablePIDF;
import frc.lib.subsystem.Periodic;
import frc.robot.OperatorDashboard;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class ExampleRollerSubsystem implements Periodic {
    private static final LoggedTunableNumber runAtVoltage = new LoggedTunableNumber("ExampleRollerSubsystem/Goal/RunAtVoltage", 3.0);

    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    private final Motor motor = Motor
            .createSparkMax(
                    "ExampleRollerSubsystem",
                    -1,
                    new CtrlSparkMaxConfig()
                            .withCurrentLimit(40)
                            .withInverted(false)
                            .withGearRatio(5)
                            .withNeutralMode(NeutralModeValue.Coast),
                    0.0,
                    MechanismSim.roller(0.01)
            )
            .withPositionGains(new LoggedTunablePIDF("ExampleRollerSubsystem/Position"))
            .withVelocityGains(new LoggedTunablePIDF("ExampleRollerSubsystem/Velocity"));

    @RequiredArgsConstructor
    public enum Goal {
        IDLE(() -> 0),
        RUN_AT_VOLTAGE(runAtVoltage::get),
        ;

        /** Should be constant for every loop cycle */
        private final DoubleSupplier volts;
    }

    @Setter
    @Getter
    private Goal goal = Goal.IDLE;

    @Getter
    private final static ExampleRollerSubsystem instance = new ExampleRollerSubsystem();

    private ExampleRollerSubsystem() {
    }

    @Override
    public void periodicBeforeCommands() {
        // Apply network inputs
        if (operatorDashboard.coastOverride.hasChanged()) {
            motor.setNeutralMode(operatorDashboard.coastOverride.get() ? NeutralModeValue.Coast : NeutralModeValue.Brake);
        }
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("ExampleRollerSubsystem/Goal", goal);
        if (DriverStation.isDisabled()) {
            motor.setVoltageRequest(0);
        } else {
            double volts = goal.volts.getAsDouble();
            Logger.recordOutput("ExampleRollerSubsystem/RequestVolts", volts);
            motor.setVoltageRequest(volts);
        }
    }
}
