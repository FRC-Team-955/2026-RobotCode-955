package frc.robot.subsystems.superstructure.turret;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
import frc.robot.RobotState;
import frc.robot.shooting.ShootingKinematics;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.superstructure.turret.TurretConstants.*;

public class Turret implements Periodic {
    private static final ShootingKinematics shootingKinematics = ShootingKinematics.get();
    private static final RobotState robotState = RobotState.get();
    private static final LoggedTunableNumber profileLookaheadTimeSec = new LoggedTunableNumber("Turret/ProfileLookaheadTimeSec", 0.15);

    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    private final MotorIO io = createIO();
    private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum Goal {
        AIM(() -> MathUtil.angleModulus(
                shootingKinematics.getShootingParameters().headingRad()
                        - robotState.getPose().getRotation().getRadians()));

        private final DoubleSupplier setpointRad;
    }

    @Setter
    @Getter
    private Turret.Goal goal = Turret.Goal.AIM;

    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
    private Double lastSetpointRad = null;
    // goalState is just for logging the profile we want to follow.
    // lookaheadState is shifted some seconds into the future, and is used for PID setpoint.
    private TrapezoidProfile.State goalState = new TrapezoidProfile.State();
    private TrapezoidProfile.State lookaheadState = new TrapezoidProfile.State();

    private final Alert motorDisconnectedAlert = new Alert("Turret motor is disconnected.", Alert.AlertType.kError);

    private static Turret instance;

    public static synchronized Turret get() {
        if (instance == null) {
            instance = new Turret();
        }

        return instance;
    }

    private Turret() {
        if (instance != null) {
            Util.error("Duplicate Turret created");
        }
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Turret", inputs);

        motorDisconnectedAlert.set(!inputs.connected);

        // Apply network inputs
        if (operatorDashboard.coastOverride.hasChanged()) {
            io.setNeutralMode(operatorDashboard.coastOverride.get() ? NeutralModeValue.Coast : NeutralModeValue.Brake);
        }

        if (gains.hasChanged()) {
            io.setPositionPIDF(gains);
        }
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Turret/Goal", goal);
        if (DriverStation.isDisabled()) {
            io.setRequest(RequestType.VoltageVolts, 0);

            // Reset states to current position
            goalState = new TrapezoidProfile.State(inputs.positionRad, 0.0);
            lookaheadState = goalState;
        } else {
            // See the comments above the lookaheadState and goalState variables for why we effectively calculate two profiles

            double setpointRad = goal.setpointRad.getAsDouble();
            setpointRad = MathUtil.clamp(setpointRad, minPositionRad, maxPositionRad);
            //            Logger.recordOutput("Turret/OriginalSetpointRad", setpointRad);
            TrapezoidProfile.State wantedState = new TrapezoidProfile.State(setpointRad, 0.0);

            if (lastSetpointRad == null || setpointRad != lastSetpointRad) {
                // Setpoint changed - shift setpoint profile into the future
                lookaheadState = profile.calculate(profileLookaheadTimeSec.get(), lookaheadState, wantedState);
            }
            lastSetpointRad = setpointRad;

            goalState = profile.calculate(Constants.loopPeriod, goalState, wantedState);
            Logger.recordOutput("Turret/ProfileSetpointRad", goalState.position);

            lookaheadState = profile.calculate(Constants.loopPeriod, lookaheadState, wantedState);
            Logger.recordOutput("Turret/LookaheadSetpointRad", lookaheadState.position);

            io.setRequest(RequestType.PositionRad, lookaheadState.position);
        }
    }

    /**
     * Intended to be plugged into component rotation in RobotMechanism
     */
    public double getPositionRad() {
        return inputs.positionRad;
    }

    /**
     * Field-relative direction the turret is pointing: chassis heading plus turret angle.
     */
    @AutoLogOutput(key = "Turret/HeadingRad")
    public double getHeadingRad() {
        return MathUtil.angleModulus(robotState.getPose().getRotation().getRadians() + inputs.positionRad);
    }

    @AutoLogOutput(key = "Turret/AtGoal")
    public boolean atGoal() {
        double value = goal.setpointRad.getAsDouble();
        return Math.abs(inputs.positionRad - value) <= positionToleranceRad;
    }

    public Command waitUntilAtGoal() {
        return Commands.waitUntil(this::atGoal);
    }

}
