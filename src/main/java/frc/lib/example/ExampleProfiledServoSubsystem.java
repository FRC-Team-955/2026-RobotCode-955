package frc.lib.example;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
import frc.robot.Constants;
import frc.robot.OperatorDashboard;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class ExampleProfiledServoSubsystem implements Periodic {
    // 0 = parallel with ground
    private static final double minPositionRad = Units.degreesToRadians(0);
    private static final double maxPositionRad = Units.degreesToRadians(90);
    private static final double initialPositionRad = maxPositionRad;

    private static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1, 3);

    private static final double positionToleranceRad = Units.degreesToRadians(10);

    private static final LoggedTunableNumber agitateSetpointDegrees = new LoggedTunableNumber("ExampleProfiledServoSubsystem/Goal/AgitateDegrees", 45);

    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    private final Motor motor = Motor
            .createTalonFX(
                    "ExampleProfiledServoSubsystem",
                    -1,
                    new TalonFXConfiguration()
                            .withMotorOutput(new MotorOutputConfigs()
                                    .withNeutralMode(NeutralModeValue.Brake)
                                    .withInverted(InvertedValue.CounterClockwise_Positive))
                            .withCurrentLimits(new CurrentLimitsConfigs()
                                    .withStatorCurrentLimit(90)
                                    .withSupplyCurrentLimit(50))
                            .withFeedback(new FeedbackConfigs()
                                    .withSensorToMechanismRatio(120)),
                    initialPositionRad,
                    MechanismSim.arm(
                            0.1,
                            Units.inchesToMeters(10),
                            minPositionRad,
                            maxPositionRad,
                            true
                    )
            )
            .withPositionGains(switch (BuildConstants.mode) {
                case REAL, REPLAY -> new LoggedTunablePIDF("ExampleProfiledServoSubsystem/PositionGains")
                        .withP(0.1);
                case SIM -> new LoggedTunablePIDF("ExampleProfiledServoSubsystem/PositionGains")
                        .withP(1.0);
            })
            .withVelocityGains(switch (BuildConstants.mode) {
                case REAL, REPLAY -> new LoggedTunablePIDF("ExampleProfiledServoSubsystem/VelocityGains")
                        .withV(0.1);
                case SIM -> new LoggedTunablePIDF("ExampleProfiledServoSubsystem/VelocityGains")
                        .withV(1.0);
            });

    @RequiredArgsConstructor
    public enum Goal {
        STOW(() -> maxPositionRad),
        DEPLAY(() -> minPositionRad),
        AGITATE(() -> Units.degreesToRadians(agitateSetpointDegrees.get())),
        ;

        /** Should be constant for every loop cycle */
        private final DoubleSupplier setpointRad;
    }

    @Setter
    @Getter
    private Goal goal = Goal.STOW;

    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
    private TrapezoidProfile.State state = new TrapezoidProfile.State();

    @Getter
    private final static ExampleProfiledServoSubsystem instance = new ExampleProfiledServoSubsystem();

    private ExampleProfiledServoSubsystem() {
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
        Logger.recordOutput("ExampleProfiledServoSubsystem/Goal", goal);
        if (DriverStation.isDisabled()) {
            motor.setVoltageRequest(0.0);

            // Reset state to current position
            state = new TrapezoidProfile.State(motor.getPositionRad(), 0.0);
        } else {
            // See the comments above the lookaheadState and goalState variables for why we calculate two profiles

            double setpointRad = goal.setpointRad.getAsDouble();
            setpointRad = MathUtil.clamp(setpointRad, minPositionRad, maxPositionRad);
            if (BuildConstants.isSimOrReplay) {
                Logger.recordOutput("ExampleProfiledServoSubsystem/OriginalSetpointRad", setpointRad);
            }
            TrapezoidProfile.State wantedState = new TrapezoidProfile.State(setpointRad, 0.0);

            state = profile.calculate(Constants.loopPeriod, state, wantedState);
            Logger.recordOutput("ExampleProfiledServoSubsystem/ProfileSetpointRad", state.position);

            motor.setMotionProfileRequest(state.position, state.velocity);
        }
    }

    @AutoLogOutput(key = "ExampleProfiledServoSubsystem/AtGoal")
    public boolean atGoal() {
        return Math.abs(motor.getPositionRad() - goal.setpointRad.getAsDouble()) <= positionToleranceRad;
    }

    public Command waitUntilAtGoal() {
        return Commands.waitUntil(this::atGoal);
    }
}
