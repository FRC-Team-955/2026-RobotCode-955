package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.EnergyLogger;
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

public class Feeder implements Periodic {
    private static final LoggedTunableNumber feedVoltage = new LoggedTunableNumber("Superstructure/Feeder/Goal/FeedVoltage", 12.0);
    private static final LoggedTunableNumber ejectVoltage = new LoggedTunableNumber("Superstructure/Feeder/Goal/EjectVoltage", -12.0);

    private final Motor motor = Motor
            .createSparkMax(
                    "Superstructure/Feeder",
                    11,
                    new CtrlSparkMaxConfig()
                            .withInverted(true)
                            .withNeutralMode(NeutralModeValue.Brake)
                            .withCurrentLimit(40)
                            .withGearRatio(3),
                    0.0,
                    MechanismSim.roller(0.01)
            );

    @RequiredArgsConstructor
    public enum Goal {
        IDLE(() -> 0),
        FEED(feedVoltage::get),
        EJECT(ejectVoltage::get),
        ;

        /** Should be constant for every loop cycle */
        private final DoubleSupplier voltageVolts;
    }

    @Setter
    @Getter
    private Goal goal = Goal.IDLE;

    @Getter
    private static final Feeder instance = new Feeder();

    private Feeder() {
    }

    @Override
    public void periodicBeforeCommands() {
        EnergyLogger.getInstance().reportPowerUsage("Feeder", motor.isConnected() ? motor.getAppliedVolts() * motor.getSupplyCurrentAmps() : 0.0);

        // Apply network inputs
        if (OperatorDashboard.getInstance().coastOverride.hasChanged()) {
            motor.setNeutralMode(OperatorDashboard.getInstance().coastOverride.get() ? NeutralModeValue.Coast : NeutralModeValue.Brake);
        }
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Superstructure/Feeder/Goal", goal);
        if (DriverStation.isDisabled()) {
            motor.setVoltageRequest(0);
        } else {
            double volts = goal.voltageVolts.getAsDouble();
            motor.setVoltageRequest(volts);
            if (BuildConstants.isSimOrReplay) {
                Logger.recordOutput("Superstructure/Feeder/RequestedVolts", volts);
            }
        }
    }

    public boolean isDisconnected() {
        return !motor.isConnected();
    }

    public Transform3d getMechanismTransform() {
        return new Transform3d(
                new Translation3d(Units.inchesToMeters(-3.451296), Units.inchesToMeters(-5.445256), Units.inchesToMeters(8.430151)),
                new Rotation3d(0.0, 0.0, Units.degreesToRadians(90.0))
        ).plus((new Transform3d(
                new Translation3d(),
                new Rotation3d(0.0, motor.getPositionRad(), 0.0)
        )));
    }
}
