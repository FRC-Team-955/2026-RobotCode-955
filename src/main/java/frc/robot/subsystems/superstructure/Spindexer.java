package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.Util;
import frc.lib.device.*;
import frc.lib.network.LoggedTunableNumber;
import frc.lib.subsystem.Periodic;
import frc.robot.BuildConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class Spindexer implements Periodic {
    private static final LoggedTunableNumber idleVoltage = new LoggedTunableNumber(" Spindexer/Goal/IdleVoltage", 0.0);

    private static final LoggedTunableNumber feedVoltage = new LoggedTunableNumber(" Spindexer/Goal/FeedVoltage", 12.0);
    private static final LoggedTunableNumber ejectVoltage = new LoggedTunableNumber(" Spindexer/Goal/EjectVoltage", -12.0);
    private static final double gearRatio = 5;
    private static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(60)
                    .withSupplyCurrentLimit(50)
            )
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(gearRatio)
            );
    private final Motor motor = new Motor("Spindexer", switch (BuildConstants.mode) {
        case REAL -> new MotorIOTalonFX(13, motorConfig, 0.0);
        case SIM -> new MotorIOTalonFXSim(motorConfig, 0.0, MechanismSim.roller(gearRatio, 0.01));
        case REPLAY -> new MotorIOReplay();
    });


    @RequiredArgsConstructor
    public enum Goal {
        IDLE(idleVoltage::get),
        FEED(feedVoltage::get),
        EJECT(ejectVoltage::get),
        ;

        /** Should be constant for every loop cycle */
        private final DoubleSupplier volts;
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
        Logger.recordOutput("Spindexer/Goal", goal);
        if (DriverStation.isDisabled()) {
            motor.setVoltageRequest(0);
        } else {
            double value = goal.volts.getAsDouble();
            //Logger.recordOutput("Spindexer/RequestValue", value);
            motor.setVoltageRequest(value);
        }
    }

    public double getPositionRad() {
        return motor.getPositionRad();
    }
}
