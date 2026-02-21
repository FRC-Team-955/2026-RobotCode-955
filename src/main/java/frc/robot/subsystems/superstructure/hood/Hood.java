package frc.robot.subsystems.superstructure.hood;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Util;
import frc.lib.motor.MotorIOInputsAutoLogged;
import frc.lib.network.LoggedTunableNumber;
import frc.lib.subsystem.Periodic;
import frc.robot.Constants;
import frc.robot.OperatorDashboard;
import frc.robot.RobotState;
import frc.robot.ShootingKinematics;
import frc.robot.subsystems.superstructure.hood.HoodIO.HoodCurrentLimitMode;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.superstructure.hood.HoodConstants.*;

public class Hood implements Periodic {
    private static final LoggedTunableNumber profileLookaheadTimeSec = new LoggedTunableNumber("Superstructure/Hood/ProfileLookaheadTimeSec", 0.15);
    private static final LoggedTunableNumber stowSetpointDegrees = new LoggedTunableNumber("Superstructure/Hood/Goal/StowDegrees", 45.0);
    private static final LoggedTunableNumber shootHubManualSetpointDegrees = new LoggedTunableNumber("Superstructure/Hood/Goal/ShootHubManualDegrees", 45.0);
    private static final LoggedTunableNumber shootTowerManualSetpointDegrees = new LoggedTunableNumber("Superstructure/Hood/Goal/ShootTowerManualDegrees", 45.0);
    private static final LoggedTunableNumber passManualSetpointDegrees = new LoggedTunableNumber("Superstructure/Hood/Goal/PassManualDegrees", 45.0);
    private static final LoggedTunableNumber ejectSetpointDegrees = new LoggedTunableNumber("Superstructure/Hood/Goal/EjectDegrees", 45.0);

    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private static final ShootingKinematics shootingKinematics = ShootingKinematics.get();
    private static final RobotState robotState = RobotState.get();

    private final HoodIO io = createIO();
    private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum Goal {
        STOW(() -> Units.degreesToRadians(stowSetpointDegrees.get())),
        SHOOT_AND_PASS_AUTOMATIC(() -> shootingKinematics.getShootingParameters().hoodAngleRad()),
        SHOOT_HUB_MANUAL(() -> Units.degreesToRadians(shootHubManualSetpointDegrees.get())),
        SHOOT_TOWER_MANUAL(() -> Units.degreesToRadians(shootTowerManualSetpointDegrees.get())),
        PASS_MANUAL(() -> Units.degreesToRadians(passManualSetpointDegrees.get())),
        EJECT(() -> Units.degreesToRadians(ejectSetpointDegrees.get())),
        ;

        private final DoubleSupplier setpointRad;
    }

    @Setter
    @Getter
    private Goal goal = Goal.STOW;

    private HoodCurrentLimitMode currentLimitMode = HoodCurrentLimitMode.NORMAL;

    public void setCurrentLimitMode(HoodCurrentLimitMode newCurrentLimitMode) {
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

    private final Alert motorDisconnectedAlert = new Alert("Hood motor is disconnected.", Alert.AlertType.kError);

    private static Hood instance;

    public static synchronized Hood get() {
        if (instance == null) {
            instance = new Hood();
        }

        return instance;
    }

    private Hood() {
        if (instance != null) {
            Util.error("Duplicate Hood created");
        }
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Superstructure/Hood", inputs);

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
        Logger.recordOutput("Superstructure/Hood/Goal", goal);
        if (DriverStation.isDisabled()) {
            io.setStopRequest();
        } else {
            // See the comments above the lookaheadState and goalState variables for why we effectively calculate two profiles

            double setpointRad = goal.setpointRad.getAsDouble();
            if (robotState.isInTrench()) {
                setpointRad = Math.min(setpointRad, maxPositionUnderTrench);
            }
            setpointRad = MathUtil.clamp(setpointRad, minPositionRad, maxPositionRad);
//            Logger.recordOutput("Superstructure/Hood/OriginalSetpointRad", setpointRad);
            TrapezoidProfile.State wantedState = new TrapezoidProfile.State(setpointRad, 0.0);

            if (lastSetpointRad == null || setpointRad != lastSetpointRad) {
                // Setpoint changed - shift setpoint profile into the future
                lookaheadState = profile.calculate(profileLookaheadTimeSec.get(), lookaheadState, wantedState);
            }
            lastSetpointRad = setpointRad;

            goalState = profile.calculate(Constants.loopPeriod, goalState, wantedState);
            Logger.recordOutput("Superstructure/Hood/ProfileSetpointRad", goalState.position);

            lookaheadState = profile.calculate(Constants.loopPeriod, lookaheadState, wantedState);
            Logger.recordOutput("Superstructure/Hood/LookaheadSetpointRad", lookaheadState.position);

            io.setPositionRequest(lookaheadState.position);
        }
    }

    public double getPositionRad() {
        return inputs.positionRad;
    }

    @AutoLogOutput(key = "Superstructure/Hood/AtGoal")
    public boolean atGoal() {
        double value = goal.setpointRad.getAsDouble();
        return Math.abs(inputs.positionRad - value) <= positionToleranceRad;
    }

    public Command waitUntilAtGoal() {
        return Commands.waitUntil(this::atGoal);
    }
}
