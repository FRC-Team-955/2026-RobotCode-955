package frc.robot.subsystems.superstructure.hood;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.PIDF;
import frc.lib.motor.MotorIO;
import frc.lib.motor.MotorIOInputsAutoLogged;
import frc.lib.motor.RequestType;
import frc.lib.network.LoggedTunableNumber;
import frc.lib.subsystem.Periodic;
import frc.robot.OperatorDashboard;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.lib.example.HoodConstants.*;

public class Hood implements Periodic {
    private static final PIDF.Tunable gainsTunable = gains.tunable("Hood/Gains");
    private static final LoggedTunableNumber deploySetpointDegrees = new LoggedTunableNumber("Hood/Goal/Deploy", -45.0);
    private static final LoggedTunableNumber profileLookaheadTimeSec = new LoggedTunableNumber("Hood/ProfileLookaheadTimeSec", 0.15);

    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();

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

    private final Alert motorDisconnectedAlert = new Alert("Hood motor is disconnected.", Alert.AlertType.kError);

    private static Hood instance;

    public static Hood get() {
        if (instance == null)
            synchronized (Hood.class) {
                instance = new Hood();
            }

        return instance;
    }

    private Hood() {
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Hood", inputs);

        motorDisconnectedAlert.set(!inputs.connected);

        // Apply network inputs
        if (operatorDashboard.coastOverride.hasChanged()) {
            io.setBrakeMode(!operatorDashboard.coastOverride.get());
        }

        gainsTunable.ifChanged(io::setPositionPIDF);
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Hood/Goal", goal);
        if (DriverStation.isDisabled()) {
            io.setRequest(RequestType.VoltageVolts, 0);
        } else {
            // See the comments above the lookaheadState and goalState variables for why we effectively calculate two profiles

            double setpointRad = goal.setpointRad.getAsDouble();
            Logger.recordOutput("Hood/OriginalSetpointRad", setpointRad);
            TrapezoidProfile.State wantedState = new TrapezoidProfile.State(setpointRad, 0.0);

            if (lastSetpointRad == null || setpointRad != lastSetpointRad) {
                // Setpoint changed - shift setpoint profile into the future
                lookaheadState = profile.calculate(profileLookaheadTimeSec.get(), lookaheadState, wantedState);
            }
            lastSetpointRad = setpointRad;

            goalState = profile.calculate(0.02, goalState, wantedState);
            Logger.recordOutput("Hood/ProfileSetpointRad", goalState.position);

            lookaheadState = profile.calculate(0.02, lookaheadState, wantedState);
            Logger.recordOutput("Hood/LookaheadSetpointRad", lookaheadState.position);

            io.setRequest(RequestType.PositionRad, lookaheadState.position);
        }
    }

    /**
     * Intended to be plugged into component rotation in RobotMechanism
     */
    public double getPositionRad() {
        return inputs.positionRad;
    }

    @AutoLogOutput(key = "Hood/AtGoal")
    public boolean atGoal() {
        double value = goal.setpointRad.getAsDouble();
        return Math.abs(inputs.positionRad - value) <= tolerances.positionToleranceRad();
    }

    public Command waitUntilAtGoal() {
        return Commands.waitUntil(this::atGoal);
    }
}
