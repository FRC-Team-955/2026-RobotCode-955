package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.Util;
import frc.lib.device.*;
import frc.lib.network.LoggedTunablePIDF;
import frc.lib.subsystem.Periodic;
import frc.robot.BuildConstants;
import frc.robot.OperatorDashboard;
import frc.robot.RobotState;
import frc.robot.shooting.ShootingKinematics;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class Hood implements Periodic {
    public static final double minPositionRad = Units.degreesToRadians(15.0);
    public static final double maxPositionRad = Units.degreesToRadians(40.0);
    public static final double initialPositionRad = minPositionRad;
    public static final double maxPositionUnderTrench = Units.degreesToRadians(30.0);

    public static final double gearRatio = 5.0 * 2.0 * (220.0 / 20.0);

    public static final LoggedTunablePIDF gains = switch (BuildConstants.mode) {
        case REAL, REPLAY -> new LoggedTunablePIDF("Superstructure/Hood/Gains")
                .withP(2.0)
                .withD(0.0)
                .withG(0.2, GravityTypeValue.Arm_Cosine)
                .withS(0.1, StaticFeedforwardSignValue.UseClosedLoopSign);
        case SIM -> new LoggedTunablePIDF("Superstructure/Hood/Gains")
                .withP(30)
                .withG(0.3, GravityTypeValue.Arm_Cosine);
    };

    public static double convertBetweenShotAngleAndHoodAngleRad(double originalAngleRad) {
        return Math.PI / 2.0 - originalAngleRad;
    }

    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private static final ShootingKinematics shootingKinematics = ShootingKinematics.get();
    private static final RobotState robotState = RobotState.get();

    private static final CtrlSparkMaxConfig motorConfig = new CtrlSparkMaxConfig()
            .withInverted(true)
            .withNeutralMode(NeutralModeValue.Brake)
            .withCurrentLimit(30)
            .withGearRatio(gearRatio)
            .withGains(gains);

    private final Motor motor = new Motor("Hood", switch (BuildConstants.mode) {
        case REAL -> new MotorIOSparkMax(10, motorConfig, initialPositionRad);
        case SIM ->
                new MotorIOSparkMaxSim(motorConfig, initialPositionRad, MechanismSim.arm(gearRatio, 0.01, Units.inchesToMeters(2), minPositionRad, maxPositionRad, true));
        case REPLAY -> new MotorIOReplay();
    });

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

    @Getter
    private boolean emergencyStopped = false;
    private final Debouncer emergencyStopDebouncer = new Debouncer(2.0, Debouncer.DebounceType.kRising);

    @Getter
    private boolean atVelocityThresholdForHoming = false;
    private final Debouncer homingVelocityDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

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

        if (!emergencyStopped) {
            if (emergencyStopDebouncer.calculate(motor.getStatorCurrentAmps() >= 20) || operatorDashboard.hoodEStop.get()) {
                motor.setVoltageRequest(0.0);
                motor.setNeutralMode(NeutralModeValue.Coast);
                emergencyStopped = true;
                operatorDashboard.hoodEStop.set(true);
            }
        } else {
            if (!operatorDashboard.hoodEStop.get()) {
                // Let operator turn off e-stop
                motor.setNeutralMode(NeutralModeValue.Brake);
                emergencyStopped = false;
                operatorDashboard.hoodEStop.set(false);
            }
        }
        emergencyStoppedAlert.set(emergencyStopped);

        atVelocityThresholdForHoming = homingVelocityDebouncer.calculate(goal == Goal.HOME && Math.abs(motor.getVelocityRadPerSec()) < 0.1);

        // Apply network inputs
        if (!emergencyStopped && operatorDashboard.coastOverride.hasChanged()) {
            motor.setNeutralMode(operatorDashboard.coastOverride.get() ? NeutralModeValue.Coast : NeutralModeValue.Brake);
        }
        if (gains.hasChanged()) {
            motor.setGains(gains);
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
            motor.setVoltageRequest(0.0);
        } else if (goal == Goal.HOME) {
            motor.setVoltageRequest(-0.5);
        } else {
            double setpointRad = goal.setpointRad.getAsDouble();
            if (robotState.isInTrench()) {
                setpointRad = Math.min(setpointRad, maxPositionUnderTrench);
            }
            setpointRad = MathUtil.clamp(setpointRad, minPositionRad, maxPositionRad);
            motor.setPositionRequest(setpointRad);
        }
    }

    public double getPositionRad() {
        return motor.getPositionRad();
    }

    public double getShotAngleRad() {
        return convertBetweenShotAngleAndHoodAngleRad(getPositionRad());
    }

    public void finishHoming() {
        motor.setEncoderPosition(initialPositionRad);
        operatorDashboard.hoodNotHomedAlert.set(false);
    }
}
