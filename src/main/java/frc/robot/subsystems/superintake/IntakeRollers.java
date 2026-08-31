package frc.robot.subsystems.superintake;

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

public class IntakeRollers implements Periodic {
    private static final LoggedTunableNumber idleVoltage = new LoggedTunableNumber("Superintake/IntakeRollers/Goal/IdleVoltage", 0.0);
    private static final LoggedTunableNumber intakeVoltage = new LoggedTunableNumber("Superintake/IntakeRollers/Goal/IntakeVoltage", 12.0);
    private static final LoggedTunableNumber ejectVoltage = new LoggedTunableNumber("Superintake/IntakeRollers/Goal/EjectVoltage", -12.0);
    private static final EnergyLogger energyLogger = EnergyLogger.get();

    private final Motor motor = Motor
            .createTalonFX(
                    "Superintake/IntakeRollers",
                    15,
                    new TalonFXConfiguration()
                            .withMotorOutput(new MotorOutputConfigs()
                                    .withNeutralMode(NeutralModeValue.Coast)
                                    .withInverted(InvertedValue.Clockwise_Positive))
                            .withCurrentLimits(new CurrentLimitsConfigs()
                                    .withStatorCurrentLimit(140)
                                    .withSupplyCurrentLimit(70))
                            .withFeedback(new FeedbackConfigs()
                                    .withSensorToMechanismRatio(3)),
                    0.0,
                    MechanismSim.roller(0.01)
            );


    @RequiredArgsConstructor
    public enum Goal {
        IDLE(idleVoltage::get),
        INTAKE(intakeVoltage::get),
        EJECT(ejectVoltage::get),
        ;

        /** Should be constant for every loop cycle */
        private final DoubleSupplier voltageVolts;
    }

    @Setter
    @Getter
    private Goal goal = Goal.IDLE;

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
        energyLogger.reportPowerUsage("IntakeRollers", motor.isConnected() ? motor.getAppliedVolts() * motor.getSupplyCurrentAmps() : 0.0);
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Superintake/IntakeRollers/Goal", goal);
        if (DriverStation.isDisabled()) {
            motor.setVoltageRequest(0);
        } else {
            double volts = goal.voltageVolts.getAsDouble();
            motor.setVoltageRequest(volts);
            if (BuildConstants.isSimOrReplay) {
                Logger.recordOutput("Superintake/IntakeRollers/RequestedVolts", volts);
            }
        }
    }

    public Transform3d getMechanismTransform() {
        return new Transform3d(
                new Translation3d(Units.inchesToMeters(19.75), 0.0, Units.inchesToMeters(8.985680)),
                new Rotation3d()
        ).plus(new Transform3d(
                new Translation3d(),
                new Rotation3d(0.0, motor.getPositionRad(), 0.0)
        ));
    }
}
