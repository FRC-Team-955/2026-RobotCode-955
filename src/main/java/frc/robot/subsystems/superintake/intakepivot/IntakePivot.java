package frc.robot.subsystems.superintake.intakepivot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Util;
import frc.lib.motor.MotorIO;
import frc.lib.motor.MotorIOInputsAutoLogged;
import frc.lib.motor.RequestType;
import frc.lib.network.LoggedTunableNumber;
import frc.lib.subsystem.Periodic;
import frc.robot.Constants;
import frc.robot.OperatorDashboard;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.superintake.intakepivot.IntakePivotConstants.*;

public class IntakePivot implements Periodic {
    private static final LoggedTunableNumber profileLookaheadTimeSec = new LoggedTunableNumber("Superintake/IntakePivot/ProfileLookaheadTimeSec", 0.15);
    private static final LoggedTunableNumber deploySetpointDegrees = new LoggedTunableNumber("Superintake/IntakePivot/Goal/DeployDegrees", -45.0);

    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    private final MotorIO io = createIO();
    private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();


    @RequiredArgsConstructor
    public enum Goal {
        STOW(() -> 0),
        DEPLOY(() -> Units.degreesToRadians(deploySetpointDegrees.get())),
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

    private final Alert motorDisconnectedAlert = new Alert("IntakePivot motor is disconnected.", Alert.AlertType.kError);

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
            io.setRequest(RequestType.VoltageVolts, 0);
        } else {
            // See the comments above the lookaheadState and goalState variables for why we effectively calculate two profiles

            double setpointRad = goal.setpointRad.getAsDouble();
            Logger.recordOutput("Superintake/IntakePivot/OriginalSetpointRad", setpointRad);
            TrapezoidProfile.State wantedState = new TrapezoidProfile.State(setpointRad, 0.0);

            if (lastSetpointRad == null || setpointRad != lastSetpointRad) {
                // Setpoint changed - shift setpoint profile into the future
                lookaheadState = profile.calculate(profileLookaheadTimeSec.get(), lookaheadState, wantedState);
            }
            lastSetpointRad = setpointRad;

            goalState = profile.calculate(Constants.loopPeriod, goalState, wantedState);
            Logger.recordOutput("Superintake/IntakePivot/ProfileSetpointRad", goalState.position);

            lookaheadState = profile.calculate(Constants.loopPeriod, lookaheadState, wantedState);
            Logger.recordOutput("Superintake/IntakePivot/LookaheadSetpointRad", lookaheadState.position);

            io.setRequest(RequestType.PositionRad, lookaheadState.position);
        }
    }

    public double getPositionRad() {
        return inputs.positionRad;
    }

    @AutoLogOutput(key = "Superintake/IntakePivot/AtGoal")
    public boolean atGoal() {
        double value = goal.setpointRad.getAsDouble();
        return Math.abs(inputs.positionRad - value) <= positionToleranceRad;
    }

    public Command waitUntilAtGoal() {
        return Commands.waitUntil(this::atGoal);
    }
}
