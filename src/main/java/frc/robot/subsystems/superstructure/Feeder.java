package frc.robot.subsystems.superstructure;

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

public class Feeder implements Periodic {
    private static final LoggedTunableNumber idleVoltage = new LoggedTunableNumber("Feeder/Goal/IdleVoltage", 0.0);
    private static final LoggedTunableNumber feedVoltage = new LoggedTunableNumber("Feeder/Goal/FeedVoltage", 12.0);
    private static final LoggedTunableNumber ejectVoltage = new LoggedTunableNumber("Feeder/Goal/EjectVoltage", -12.0);

    private static final double gearRatio = 5;
    private static final CtrlSparkMaxConfig motorConfig = new CtrlSparkMaxConfig()
            .withInverted(true).withNeutralMode(NeutralModeValue.Brake).withCurrentLimit(40).withGearRatio(gearRatio);


    //.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40)
    //.voltageCompensation(12.0).
    //encoder.positionConversionFactor(2 * Math.PI / gearRatio) // Rotor Rotations -> Wheel Radians
    //.velocityConversionFactor((2 * Math.PI) / 60.0 / gearRatio) // Rotor RPM -> Wheel Rad/Sec
    //.uvwMeasurementPeriod(10)
    //.uvwAverageDepth(2);

    private final Motor motor = new Motor("Feeder", switch (BuildConstants.mode) {
        case REAL -> new MotorIOSparkMax(11, motorConfig, 0.0);
        case SIM -> new MotorIOSparkMaxSim(motorConfig, 0.0, MechanismSim.roller(gearRatio, 0.01));
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
    public void periodicAfterCommands() {
        Logger.recordOutput("Feeder/Goal", goal);
        if (DriverStation.isDisabled()) {
            motor.setVoltageRequest(0);
        } else {
            double value = goal.volts.getAsDouble();
            //Logger.recordOutput("Feeder/RequestValue", value);
            motor.setVoltageRequest(value);
        }
    }

    public double getPositionRad() {
        return motor.getPositionRad();
    }
}
