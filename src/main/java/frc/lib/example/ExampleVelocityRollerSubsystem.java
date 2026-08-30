package frc.lib.example;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.devices.motor.MechanismSim;
import frc.lib.devices.motor.Motor;
import frc.lib.network.LoggedTunableNumber;
import frc.lib.network.LoggedTunablePIDF;
import frc.lib.subsystem.Periodic;
import frc.robot.BuildConstants;
import frc.robot.OperatorDashboard;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class ExampleVelocityRollerSubsystem implements Periodic {
    private static final double velocityToleranceRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(10);

    private static final LoggedTunableNumber rollRPM = new LoggedTunableNumber("ExampleVelocityRollerSubsystem/Goal/RollRPM", 100);

    private final Motor motor = Motor
            .createTalonFX(
                    "ExampleVelocityRollerSubsystem",
                    -1,
                    new TalonFXConfiguration()
                            .withMotorOutput(new MotorOutputConfigs()
                                    .withNeutralMode(NeutralModeValue.Coast)
                                    .withInverted(InvertedValue.CounterClockwise_Positive))
                            .withCurrentLimits(new CurrentLimitsConfigs()
                                    .withStatorCurrentLimit(90)
                                    .withSupplyCurrentLimit(50))
                            .withFeedback(new FeedbackConfigs()
                                    .withSensorToMechanismRatio(5)),
                    0.0,
                    MechanismSim.roller(0.01)
            )
            .withVelocityGains(switch (BuildConstants.mode) {
                case REAL, REPLAY -> new LoggedTunablePIDF("ExampleVelocityRollerSubsystem/Gains")
                        .withV(0.1);
                case SIM -> new LoggedTunablePIDF("ExampleVelocityRollerSubsystem/Gains")
                        .withV(1.0);
            });

    @RequiredArgsConstructor
    public enum Goal {
        IDLE(() -> 0),
        ROLL(() -> Units.rotationsPerMinuteToRadiansPerSecond(rollRPM.get())),
        ;

        /** Should be constant for every loop cycle */
        private final DoubleSupplier setpointRadPerSec;
    }

    @Setter
    @Getter
    private Goal goal = Goal.IDLE;

    @Getter
    private final static ExampleVelocityRollerSubsystem instance = new ExampleVelocityRollerSubsystem();

    private ExampleVelocityRollerSubsystem() {
    }

    @Override
    public void periodicBeforeCommands() {
        // Apply network inputs
        if (OperatorDashboard.getInstance().coastOverride.hasChanged()) {
            motor.setNeutralMode(OperatorDashboard.getInstance().coastOverride.get() ? NeutralModeValue.Coast : NeutralModeValue.Brake);
        }
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("ExampleVelocityRollerSubsystem/Goal", goal);
        if (DriverStation.isDisabled() || goal == Goal.IDLE) {
            motor.setVoltageRequest(0);
        } else {
            double setpointRadPerSec = goal.setpointRadPerSec.getAsDouble();
            motor.setVelocityRequest(setpointRadPerSec);
            if (BuildConstants.isSimOrReplay) {
                Logger.recordOutput("ExampleVelocityRollerSubsystem/SetpointRadPerSec", setpointRadPerSec);
            }
        }
    }

    @AutoLogOutput(key = "ExampleVelocityRollerSubsystem/AtGoal")
    public boolean atGoal() {
        return Math.abs(motor.getVelocityRadPerSec() - goal.setpointRadPerSec.getAsDouble()) <= velocityToleranceRadPerSec;
    }

    public Command waitUntilAtGoal() {
        return Commands.waitUntil(this::atGoal);
    }
}
