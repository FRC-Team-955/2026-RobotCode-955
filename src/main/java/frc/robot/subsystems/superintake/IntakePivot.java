package frc.robot.subsystems.superintake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.EnergyLogger;
import frc.lib.Util;
import frc.lib.devices.motor.MechanismSim;
import frc.lib.devices.motor.Motor;
import frc.lib.network.LoggedTunableNumber;
import frc.lib.network.LoggedTunablePIDF;
import frc.lib.subsystem.Periodic;
import frc.robot.BuildConstants;
import frc.robot.Constants;
import frc.robot.OperatorDashboard;
import frc.robot.RobotState;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class IntakePivot implements Periodic {
    private static final double minPositionRad = Units.degreesToRadians(16.827716);
    private static final double maxPositionRad = Units.degreesToRadians(95.554559);
    private static final double initialPositionRad = maxPositionRad;
    private static final double maxPositionUnderTrench = Units.degreesToRadians(20.0);
    private static final double thresholdForLoweringUnderTrench = minPositionRad + Units.degreesToRadians(45.0);

    private static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(5, 15);

    private static final LoggedTunableNumber profileLookaheadTimeSec = new LoggedTunableNumber("Superintake/IntakePivot/ProfileLookaheadTimeSec", 0.15);
    private static final LoggedTunableNumber stowSetpointDegrees = new LoggedTunableNumber("Superintake/IntakePivot/Goal/StowDegrees", 70.0);

    private static final EnergyLogger energyLogger = EnergyLogger.get();
    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private static final RobotState robotState = RobotState.get();

    private final Motor motor = Motor
            .createTalonFX(
                    "Superintake/IntakePivot",
                    14,
                    new TalonFXConfiguration()
                            .withMotorOutput(new MotorOutputConfigs()
                                    .withNeutralMode(NeutralModeValue.Coast)
                                    .withInverted(InvertedValue.CounterClockwise_Positive))
                            .withCurrentLimits(new CurrentLimitsConfigs()
                                    .withStatorCurrentLimit(50)
                                    .withSupplyCurrentLimit(50))
                            .withFeedback(new FeedbackConfigs()
                                    .withSensorToMechanismRatio(5.0 * 5.0 * 2.0)),
                    initialPositionRad,
                    MechanismSim.arm(
                            0.0768892879,
                            Units.inchesToMeters(10),
                            minPositionRad,
                            maxPositionRad,
                            true
                    )
            )
            .withPositionGains(switch (BuildConstants.mode) {
                case REAL, REPLAY -> new LoggedTunablePIDF("Superintake/IntakePivot/Gains")
                        .withP(8.0)
                        .withG(0.5, GravityTypeValue.Arm_Cosine)
                        .withS(0.0, StaticFeedforwardSignValue.UseClosedLoopSign);
                case SIM -> new LoggedTunablePIDF("Superintake/IntakePivot/Gains")
                        .withP(20.0)
                        .withG(2.65, GravityTypeValue.Arm_Cosine);
            });


    @RequiredArgsConstructor
    public enum Goal {
        STOW(() -> Units.degreesToRadians(stowSetpointDegrees.get())),
        DEPLOY(() -> minPositionRad),
        HOME(null),
        HOME_FINALIZE(null),
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
    private TrapezoidProfile.State goalState = new TrapezoidProfile.State(initialPositionRad, 0.0);
    private TrapezoidProfile.State lookaheadState = new TrapezoidProfile.State(initialPositionRad, 0.0);

    @Getter
    private boolean atVelocityThresholdForHoming = false;
    private final Debouncer homingVelocityDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

    private static IntakePivot instance;

    public static synchronized IntakePivot get() {
        if (instance == null) {
            instance = new IntakePivot();
        }

        return instance;
    }

    private IntakePivot() {
        if (instance != null) {
            Util.error("Duplicate IntakePivot created");
        }
    }

    @Override
    public void periodicBeforeCommands() {
        if (!motor.isEmergencyStopped()) {
            if (operatorDashboard.intakePivotEStop.get()) {
                motor.emergencyStop();
                operatorDashboard.intakePivotEStop.set(true);
            }
        } else {
            if (!operatorDashboard.intakePivotEStop.get()) {
                // Let operator turn off e-stop
                motor.undoEmergencyStop();
                operatorDashboard.intakePivotEStop.set(false);
            }
        }

        atVelocityThresholdForHoming = homingVelocityDebouncer.calculate(goal == Goal.HOME && Math.abs(motor.getVelocityRadPerSec()) < 0.1);
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Superintake/IntakePivot/Goal", goal);

        // Turn off E-stop when homing
        if (goal == Goal.HOME) {
            motor.undoEmergencyStop();
        }

        if (DriverStation.isDisabled() || motor.isEmergencyStopped() || goal == Goal.HOME_FINALIZE) {
            motor.setVoltageRequest(0.0);

            lastSetpointRad = null;
            // Reset states to current position
            goalState = new TrapezoidProfile.State(motor.getPositionRad(), 0.0);
            lookaheadState = goalState;
        } else if (goal == Goal.HOME) {
            motor.setVoltageRequest(2.0);
        } else {
            // See the comments above the lookaheadState and goalState variables for why we calculate two profiles

            boolean isInTrench = robotState.isInTrench(robotState.getTranslation().
                    plus(getMechanismTransform().getTranslation().toTranslation2d()));
            Logger.recordOutput("Superintake/IntakePivot/IsInTrench", isInTrench);

            double setpointRad = goal.setpointRad.getAsDouble();
            if (isInTrench) {
                if (motor.getPositionRad() < thresholdForLoweringUnderTrench) {
                    setpointRad = Math.min(setpointRad, maxPositionUnderTrench);
                } else {
                    setpointRad = maxPositionRad;
                }
            }
            setpointRad = MathUtil.clamp(setpointRad, minPositionRad, maxPositionRad);
            if (BuildConstants.isSimOrReplay)
                Logger.recordOutput("Superintake/IntakePivot/OriginalSetpointRad", setpointRad);
            TrapezoidProfile.State wantedState = new TrapezoidProfile.State(setpointRad, 0.0);

            if (lastSetpointRad == null || setpointRad != lastSetpointRad) {
                // Setpoint changed - shift setpoint profile into the future
                lookaheadState = profile.calculate(profileLookaheadTimeSec.get(), lookaheadState, wantedState);
            }
            lastSetpointRad = setpointRad;

            goalState = profile.calculate(Constants.loopPeriod, goalState, wantedState);
            if (BuildConstants.isSimOrReplay)
                Logger.recordOutput("Superintake/IntakePivot/ProfileSetpointRad", goalState.position);

            lookaheadState = profile.calculate(Constants.loopPeriod, lookaheadState, wantedState);
            if (BuildConstants.isSimOrReplay)
                Logger.recordOutput("Superintake/IntakePivot/LookaheadSetpointRad", lookaheadState.position);

            motor.setPositionRequest(lookaheadState.position);
        }
    }

    public void finishHoming() {
        motor.setEncoderPosition(initialPositionRad);
        operatorDashboard.intakePivotNotHomedAlert.set(false);
    }

    public Transform3d getMechanismTransform() {
        return new Transform3d(
                new Translation3d(Units.inchesToMeters(10.0), 0.0, Units.inchesToMeters(6.25)),
                new Rotation3d(0.0, Units.degreesToRadians(90.0), 0.0)).plus(new Transform3d(
                new Translation3d(),
                new Rotation3d(0.0, -motor.getPositionRad(), 0.0)
        ));

    }
}
