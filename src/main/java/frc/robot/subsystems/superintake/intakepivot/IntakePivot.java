package frc.robot.subsystems.superintake.intakepivot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.Util;
import frc.lib.device.*;
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
    private static final LoggedTunableNumber profileLookaheadTimeSec = new LoggedTunableNumber("IntakePivot/ProfileLookaheadTimeSec", 0.15);
    private static final LoggedTunableNumber stowSetpointDegrees = new LoggedTunableNumber("IntakePivot/Goal/StowDegrees", 70.0);

    private static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(5, 15);

    private static final double minPositionRad = Units.degreesToRadians(16.827716);
    private static final double maxPositionRad = Units.degreesToRadians(95.554559);
    private static final double initialPositionRad = minPositionRad;
    private static final double maxPositionUnderTrench = Units.degreesToRadians(20.0);
    private static final double tresholdForLoweringUnderTrench = minPositionRad + Units.degreesToRadians(45.0);


    static final double gearRatio = 5.0 * 5.0 * 2.0;
    private static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(30)
                    .withSupplyCurrentLimit(30)
            )
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(gearRatio)
            );


    static final LoggedTunablePIDF gains = switch (BuildConstants.mode) {
        case REAL, REPLAY -> new LoggedTunablePIDF("IntakePivot/Gains")
                .withP(8.0)
                .withG(0.5, GravityTypeValue.Arm_Cosine)
                .withS(0.0, StaticFeedforwardSignValue.UseClosedLoopSign);
        case SIM -> new LoggedTunablePIDF("IntakePivot/Gains")
                .withP(20.0)
                .withG(2.65, GravityTypeValue.Arm_Cosine);
    };


    private final Motor motor = new Motor("IntakePivot", gains, switch (BuildConstants.mode) {
        case REAL -> new MotorIOTalonFX(14, motorConfig, initialPositionRad);
        case SIM -> new MotorIOTalonFXSim(motorConfig, initialPositionRad, MechanismSim.arm(
                gearRatio, 0.0768892879, Units.inchesToMeters(10), minPositionRad,
                maxPositionRad,
                true));
        case REPLAY -> new MotorIOReplay();
    });

    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private static final RobotState robotState = RobotState.get();


    @RequiredArgsConstructor
    public enum Goal {
        STOW(() -> Units.degreesToRadians(stowSetpointDegrees.get())),
        DEPLOY(() -> minPositionRad),
        HOME(null),
        ;

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
    public void periodicAfterCommands() {
        Logger.recordOutput("IntakePivot/Goal", goal);
        if (DriverStation.isDisabled()) {
            motor.setVoltageRequest(0.0);

            // Reset states to current position
            goalState = new TrapezoidProfile.State(motor.getPositionRad(), 0.0);
            lookaheadState = goalState;
        } else if (goal == Goal.HOME) {
            motor.setVoltageRequest(-2.0);
        } else {
            // See the comments above the lookaheadState and goalState variables for why we effectively calculate two profiles

            double setpointRad = goal.setpointRad.getAsDouble();
            if (robotState.isInTrench()) {
                if (getPositionRad() < tresholdForLoweringUnderTrench) {
                    setpointRad = Math.min(setpointRad, maxPositionUnderTrench);
                } else {
                    setpointRad = maxPositionRad;
                }
            }
            setpointRad = MathUtil.clamp(setpointRad, minPositionRad, maxPositionRad);
            //Logger.recordOutput("Superintake/IntakePivot/OriginalSetpointRad", setpointRad);
            TrapezoidProfile.State wantedState = new TrapezoidProfile.State(setpointRad, 0.0);

            if (lastSetpointRad == null || setpointRad != lastSetpointRad) {
                // Setpoint changed - shift setpoint profile into the future
                lookaheadState = profile.calculate(profileLookaheadTimeSec.get(), lookaheadState, wantedState);
            }
            lastSetpointRad = setpointRad;

            goalState = profile.calculate(Constants.loopPeriod, goalState, wantedState);
            //Logger.recordOutput("Superintake/IntakePivot/ProfileSetpointRad", goalState.position);

            lookaheadState = profile.calculate(Constants.loopPeriod, lookaheadState, wantedState);
            //Logger.recordOutput("Superintake/IntakePivot/LookaheadSetpointRad", lookaheadState.position);

            motor.setPositionRequest(lookaheadState.position);
        }
    }

    public double getPositionRad() {
        return motor.getPositionRad();
    }

    public boolean isCurrentAtThresholdForHoming() {
        return motor.getStatorCurrentAmps() >= 10.0;
    }

    public void finishHoming() {
        motor.setEncoderPosition(initialPositionRad);
        operatorDashboard.intakePivotNotHomedAlert.set(false);
    }
}
