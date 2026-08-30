package frc.lib.example;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.devices.motor.CtrlSparkMaxConfig;
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

public class ExampleLookaheadServoSubsystem implements Periodic {
    // 0 = parallel with ground
    private static final double minPositionRad = Units.degreesToRadians(0);
    private static final double maxPositionRad = Units.degreesToRadians(90);
    private static final double initialPositionRad = minPositionRad;

    private static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1, 3);

    private static final double positionToleranceRad = Units.degreesToRadians(10);

    private static final LoggedTunableNumber deploySetpointDegrees = new LoggedTunableNumber("ExampleLookaheadServoSubsystem/Goal/Deploy", -45.0);
    private static final LoggedTunableNumber profileLookaheadTimeSec = new LoggedTunableNumber("ExampleLookaheadServoSubsystem/ProfileLookaheadTimeSec", 0.15);

    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    private final Motor motor = Motor
            .createSparkMax(
                    "ExampleLookaheadServoSubsystem",
                    -1,
                    new CtrlSparkMaxConfig()
                            .withNeutralMode(NeutralModeValue.Brake)
                            .withInverted(false)
                            .withGearRatio(120)
                            .withCurrentLimit(40),
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
                case REAL, REPLAY -> new LoggedTunablePIDF("ExampleLookaheadServoSubsystem/Gains")
                        .withP(1.0);
                case SIM -> new LoggedTunablePIDF("ExampleLookaheadServoSubsystem/Gains")
                        .withP(10.0);
            });

    @RequiredArgsConstructor
    public enum Goal {
        STOW(() -> 0),
        DEPLOY(() -> Units.degreesToRadians(deploySetpointDegrees.get())),
        ;

        /** Should be constant for every loop cycle */
        private final DoubleSupplier setpointRad;
    }

    @Setter
    @Getter
    private Goal goal = Goal.STOW;

    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
    private Double lastSetpointRad = null;
    // goalState is just for logging the profile we want to follow.
    // lookaheadState is shifted some seconds into the future, and is used for PID setpoint.
    private TrapezoidProfile.State goalState = new TrapezoidProfile.State();
    private TrapezoidProfile.State lookaheadState = new TrapezoidProfile.State();

    @Getter
    private final static ExampleLookaheadServoSubsystem instance = new ExampleLookaheadServoSubsystem();

    private ExampleLookaheadServoSubsystem() {
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
        Logger.recordOutput("ExampleLookaheadServoSubsystem/Goal", goal);
        if (DriverStation.isDisabled()) {
            motor.setVoltageRequest(0.0);

            lastSetpointRad = null;
            // Reset states to current position
            goalState = new TrapezoidProfile.State(motor.getPositionRad(), 0.0);
            lookaheadState = goalState;
        } else {
            // See the comments above the lookaheadState and goalState variables for why we calculate two profiles

            double setpointRad = goal.setpointRad.getAsDouble();
            setpointRad = MathUtil.clamp(setpointRad, minPositionRad, maxPositionRad);
            if (BuildConstants.isSimOrReplay) {
                Logger.recordOutput("ExampleLookaheadServoSubsystem/OriginalSetpointRad", setpointRad);
            }
            TrapezoidProfile.State wantedState = new TrapezoidProfile.State(setpointRad, 0.0);

            if (lastSetpointRad == null || setpointRad != lastSetpointRad) {
                // Setpoint changed - shift setpoint profile into the future
                lookaheadState = profile.calculate(profileLookaheadTimeSec.get(), lookaheadState, wantedState);
            }
            lastSetpointRad = setpointRad;

            goalState = profile.calculate(Constants.loopPeriod, goalState, wantedState);
            Logger.recordOutput("ExampleLookaheadServoSubsystem/ProfileSetpointRad", goalState.position);

            lookaheadState = profile.calculate(Constants.loopPeriod, lookaheadState, wantedState);
            Logger.recordOutput("ExampleLookaheadServoSubsystem/LookaheadSetpointRad", lookaheadState.position);

            motor.setPositionRequest(lookaheadState.position);
        }
    }

    @AutoLogOutput(key = "ExampleLookaheadServoSubsystem/AtGoal")
    public boolean atGoal() {
        return Math.abs(motor.getPositionRad() - goal.setpointRad.getAsDouble()) <= positionToleranceRad;
    }

    public Command waitUntilAtGoal() {
        return Commands.waitUntil(this::atGoal);
    }
}
