package frc.robot.subsystems.superstructure.hood;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.Util;
import frc.lib.motor.MotorIOInputsAutoLogged;
import frc.lib.network.LoggedTunableNumber;
import frc.lib.subsystem.Periodic;
import frc.robot.OperatorDashboard;
import frc.robot.RobotState;
import frc.robot.shooting.ShootingKinematics;
import frc.robot.subsystems.superstructure.hood.HoodIO.HoodCurrentLimitMode;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.superstructure.hood.HoodConstants.*;

public class Hood implements Periodic {
    private static final LoggedTunableNumber profileLookaheadTimeSec = new LoggedTunableNumber("Superstructure/Hood/ProfileLookaheadTimeSec", 0.15);

    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private static final ShootingKinematics shootingKinematics = ShootingKinematics.get();
    private static final RobotState robotState = RobotState.get();

    private final HoodIO io = createIO();
    private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum Goal {
        SHOOT(() -> convertBetweenShotAngleAndHoodAngleRad(shootingKinematics.getShootingParameters().angleRad())),
        HOME(null),
        ;

        private final DoubleSupplier setpointRad;
    }

    @Setter
    @Getter
    private Goal goal = Goal.SHOOT;

    private HoodCurrentLimitMode currentLimitMode = HoodCurrentLimitMode.NORMAL;

    public void setCurrentLimitMode(HoodCurrentLimitMode newCurrentLimitMode) {
        if (currentLimitMode != newCurrentLimitMode) {
            currentLimitMode = newCurrentLimitMode;
            io.setCurrentLimit(currentLimitMode);
        }
    }

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
            io.setVoltageRequest(0.0);
        } else if (goal == Goal.HOME) {
            io.setVoltageRequest(-0.5);
        } else {
            double setpointRad = goal.setpointRad.getAsDouble();
            if (robotState.isInTrench()) {
                setpointRad = Math.min(setpointRad, maxPositionUnderTrench);
            }
            setpointRad = MathUtil.clamp(setpointRad, minPositionRad, maxPositionRad);
            Logger.recordOutput("Superstructure/Hood/SetpointRad", setpointRad);
            io.setPositionRequest(setpointRad);
        }
    }

    public double getPositionRad() {
        return inputs.positionRad;
    }

    public double getShotAngleRad() {
        return convertBetweenShotAngleAndHoodAngleRad(getPositionRad());
    }

    public boolean isCurrentAtThresholdForHoming() {
        return inputs.currentAmps >= 10.0;
    }

    public void finishHoming() {
        io.setEncoderPositionToInitial();
        operatorDashboard.hoodNotHomedAlert.set(false);
    }
}
