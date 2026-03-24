package frc.robot.subsystems.superstructure.hood;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.Util;
import frc.lib.motor.MotorIOInputsAutoLogged;
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
    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private static final ShootingKinematics shootingKinematics = ShootingKinematics.get();
    private static final RobotState robotState = RobotState.get();
    private final HoodIO io = createIO();
    private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum Goal {
        STOW(() -> minPositionRad),
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

    @Getter
    private boolean emergencyStopped = false;
    private final Debouncer emergencyStopDebouncer = new Debouncer(2.0, Debouncer.DebounceType.kRising);

    @Getter
    private boolean atVelocityThresholdForHoming = false;
    private final Debouncer homingVelocityDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

    private final Alert motorDisconnectedAlert = new Alert("Hood motor is disconnected.", Alert.AlertType.kError);
    public final Alert highTemperatureAlert = new Alert("Hood motor temperature is high.", Alert.AlertType.kWarning);
    private final Alert emergencyStoppedAlert = new Alert("Hood is E-stopped!", Alert.AlertType.kError);

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
        highTemperatureAlert.set(inputs.temperatureCelsius > 50);

        if (!emergencyStopped) {
            if (emergencyStopDebouncer.calculate(inputs.currentAmps >= 20) || operatorDashboard.hoodEStop.get()) {
                io.setVoltageRequest(0.0);
                io.setNeutralMode(NeutralModeValue.Coast);
                emergencyStopped = true;
                operatorDashboard.hoodEStop.set(true);
            }
        } else {
            if (!operatorDashboard.hoodEStop.get()) {
                // Let operator turn off e-stop
                io.setNeutralMode(NeutralModeValue.Brake);
                emergencyStopped = false;
                operatorDashboard.hoodEStop.set(false);
            }
        }
        emergencyStoppedAlert.set(emergencyStopped);

        atVelocityThresholdForHoming = homingVelocityDebouncer.calculate(goal == Goal.HOME && Math.abs(inputs.velocityRadPerSec) < 0.1);

        // Apply network inputs
        if (!emergencyStopped && operatorDashboard.coastOverride.hasChanged()) {
            io.setNeutralMode(operatorDashboard.coastOverride.get() ? NeutralModeValue.Coast : NeutralModeValue.Brake);
        }

        if (gains.hasChanged()) {
            io.setPositionPIDF(gains);
        }
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Superstructure/Hood/Goal", goal);

        // Turn off E-stop when homing
        if (goal == Goal.HOME) {
            emergencyStopped = false;
        }

        if (DriverStation.isDisabled() || emergencyStopped) {
            io.setVoltageRequest(0.0);
        } else if (goal == Goal.HOME) {
            io.setVoltageRequest(-0.5);
        } else {
            double setpointRad = goal.setpointRad.getAsDouble();
            boolean isInTrench = robotState.isInTrench(robotState.getTranslation().
                    plus(hoodTransform().getTranslation().toTranslation2d()));
            Logger.recordOutput("Superstructure/Hood/IsInTrench", isInTrench);
            if (isInTrench) {
                setpointRad = Math.min(setpointRad, maxPositionUnderTrench);
            }
            setpointRad = MathUtil.clamp(setpointRad, minPositionRad, maxPositionRad);
            //Logger.recordOutput("Superstructure/Hood/SetpointRad", setpointRad);
            io.setPositionRequest(setpointRad);
        }
    }

    public double getPositionRad() {
        return inputs.positionRad;
    }

    public double getShotAngleRad() {
        return convertBetweenShotAngleAndHoodAngleRad(getPositionRad());
    }

    public void finishHoming() {
        io.setEncoderPositionToInitial();
        operatorDashboard.hoodNotHomedAlert.set(false);
    }

    public Transform3d hoodTransform() {
        return new Transform3d(
                new Translation3d(Units.inchesToMeters(-6.910046), Units.inchesToMeters(-9.109744), Units.inchesToMeters(12.861381)),
                new Rotation3d(0.0, Units.degreesToRadians(-90.0), Math.PI)
        ).plus(new Transform3d(
                new Translation3d(),
                new Rotation3d(0.0, getPositionRad(), 0.0)
        ));
    }
}
