package frc.robot.subsystems.superintake.intakepivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.Util;
import frc.lib.motor.MotorIOInputsAutoLogged;
import frc.lib.network.LoggedTunableNumber;
import frc.lib.subsystem.Periodic;
import frc.robot.BuildConstants;
import frc.robot.Constants;
import frc.robot.OperatorDashboard;
import frc.robot.RobotState;
import frc.robot.subsystems.superintake.intakepivot.IntakePivotIO.IntakePivotCurrentLimitMode;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.superintake.intakepivot.IntakePivotConstants.*;

public class IntakePivot implements Periodic {
    private static final LoggedTunableNumber profileLookaheadTimeSec = new LoggedTunableNumber("Superintake/IntakePivot/ProfileLookaheadTimeSec", 0.15);
    private static final LoggedTunableNumber stowSetpointDegrees = new LoggedTunableNumber("Superintake/IntakePivot/Goal/StowDegrees", 70.0);


    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private static final RobotState robotState = RobotState.get();

    private final IntakePivotIO io = createIO();
    private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();


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

    private IntakePivotCurrentLimitMode currentLimitMode = IntakePivotCurrentLimitMode.NORMAL;

    public void setCurrentLimitMode(IntakePivotCurrentLimitMode newCurrentLimitMode) {
        if (currentLimitMode != newCurrentLimitMode) {
            currentLimitMode = newCurrentLimitMode;
            io.setCurrentLimit(currentLimitMode);
        }
    }

    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
    private Double lastSetpointRad = null;
    // goalState is just for logging the profile we want to follow.
    // lookaheadState is shifted some seconds into the future, and is used for PID setpoint.
    private TrapezoidProfile.State goalState = new TrapezoidProfile.State();
    private TrapezoidProfile.State lookaheadState = new TrapezoidProfile.State();

    private final Alert motorDisconnectedAlert = new Alert("Intake pivot motor is disconnected.", Alert.AlertType.kError);

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
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Superintake/IntakePivot", inputs);

        motorDisconnectedAlert.set(!inputs.connected);

        if (gains.hasChanged()) {
            io.setPositionPIDF(gains);
        }
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Superintake/IntakePivot/Goal", goal);

        if (DriverStation.isDisabled()) {
            io.setVoltageRequest(0.0);

            // Reset states to current position
            goalState = new TrapezoidProfile.State(inputs.positionRad, 0.0);
            lookaheadState = goalState;
        } else if (goal == Goal.HOME) {
            io.setVoltageRequest(-2.0);
        } else {
            // See the comments above the lookaheadState and goalState variables for why we effectively calculate two profiles
            boolean isInTrench = robotState.isInTrench(robotState.getTranslation().
                    plus(intakePivotTransform().getTranslation().toTranslation2d()));
            Logger.recordOutput("Superintake/IntakePivot/IsInTrench", isInTrench);

            double setpointRad = goal.setpointRad.getAsDouble();
            if (isInTrench) {
                if (getPositionRad() < tresholdForLoweringUnderTrench) {
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

            io.setPositionRequest(lookaheadState.position);
        }
    }

    public double getPositionRad() {
        return inputs.positionRad;
    }

    public boolean isCurrentAtThresholdForHoming() {
        return inputs.currentAmps >= 10.0;
    }

    public void finishHoming() {
        io.setEncoderPositionToInitial();
        operatorDashboard.intakePivotNotHomedAlert.set(false);
    }

    public Transform3d intakePivotTransform() {
        return new Transform3d(
                new Translation3d(Units.inchesToMeters(10.0), 0.0, Units.inchesToMeters(6.25)),
                new Rotation3d(0.0, Units.degreesToRadians(90.0), 0.0)).plus(new Transform3d(
                new Translation3d(),
                new Rotation3d(0.0, -getPositionRad(), 0.0)
        ));

    }
}
