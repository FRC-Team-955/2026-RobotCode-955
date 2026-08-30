package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.EnergyLogger;
import frc.lib.devices.motor.CtrlSparkMaxConfig;
import frc.lib.devices.motor.MechanismSim;
import frc.lib.devices.motor.Motor;
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
    private static final double minPositionRad = Units.degreesToRadians(15.0);
    public static final double maxPositionRad = Units.degreesToRadians(40.0);
    private static final double initialPositionRad = minPositionRad;
    private static final double maxPositionUnderTrench = Units.degreesToRadians(30.0);

    /**
     * Hood angle for vertical shot is 0°
     * Shooting angle for vertical shot is 90°
     * Shooting angle for 15° from vertical is 75°
     * Therefore, hood angle = 90° - shooting angle
     * and shooting angle = 90° - hood angle
     */
    public static double convertBetweenShotAngleAndHoodAngleRad(double originalAngleRad) {
        return Math.PI / 2.0 - originalAngleRad;
    }

    private static final OperatorDashboard operatorDashboard = OperatorDashboard.getInstance();
    private static final ShootingKinematics shootingKinematics = ShootingKinematics.getInstance();
    private static final RobotState robotState = RobotState.getInstance();
    private static final EnergyLogger energyLogger = EnergyLogger.getInstance();

    private final Motor motor = Motor.createSparkMax(
                    "Superstructure/Hood",
                    10,
                    new CtrlSparkMaxConfig()
                            .withInverted(true)
                            .withCurrentLimit(30)
                            .withGearRatio(5.0 * 2.0 * (220.0 / 20.0))
                            .withNeutralMode(NeutralModeValue.Brake),
                    initialPositionRad,
                    MechanismSim.arm(
                            0.1,
                            Units.inchesToMeters(2),
                            minPositionRad,
                            maxPositionRad,
                            true
                    )
            )
            .withPositionGains(switch (BuildConstants.mode) {
                case REAL, REPLAY -> new LoggedTunablePIDF("Superstructure/Hood/Gains")
                        .withP(2.0)
                        .withD(0.0)
                        .withG(0.2, GravityTypeValue.Arm_Cosine)
                        .withS(0.1, StaticFeedforwardSignValue.UseClosedLoopSign);
                case SIM -> new LoggedTunablePIDF("Superstructure/Hood/Gains")
                        .withP(2)
                        .withG(1, GravityTypeValue.Arm_Cosine);
            });

    @RequiredArgsConstructor
    public enum Goal {
        STOW(() -> minPositionRad),
        SHOOT(() -> convertBetweenShotAngleAndHoodAngleRad(shootingKinematics.getShootingParameters().angleRad())),
        HOME(null),
        HOME_FINALIZE(null),
        ;

        private final DoubleSupplier setpointRad;
    }

    @Setter
    @Getter
    private Goal goal = Goal.SHOOT;

    private final Debouncer emergencyStopDebouncer = new Debouncer(2.0, Debouncer.DebounceType.kRising);

    @Getter
    private boolean atVelocityThresholdForHoming = false;
    private final Debouncer homingVelocityDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

    @Getter
    private static final Hood instance = new Hood();

    private Hood() {
    }

    @Override
    public void periodicBeforeCommands() {
        energyLogger.reportPowerUsage("Hood", motor.isConnected() ? motor.getAppliedVolts() * motor.getSupplyCurrentAmps() : 0.0);

        boolean shouldEmergencyStop = emergencyStopDebouncer.calculate(motor.getStatorCurrentAmps() >= 20);
        if (!motor.isEmergencyStopped()) {
            if ((shouldEmergencyStop || operatorDashboard.hoodEStop.get()) && !BuildConstants.isSim) {
                motor.emergencyStop(NeutralModeValue.Coast);
                operatorDashboard.hoodEStop.set(true);
            }
        } else {
            if (!operatorDashboard.hoodEStop.get()) {
                motor.undoEmergencyStop(NeutralModeValue.Brake);
                operatorDashboard.hoodEStop.set(false);
            }
        }

        atVelocityThresholdForHoming = homingVelocityDebouncer.calculate(goal == Goal.HOME && Math.abs(motor.getVelocityRadPerSec()) < 0.1);

        // Apply network inputs
        if (!motor.isEmergencyStopped() && operatorDashboard.coastOverride.hasChanged()) {
            motor.setNeutralMode(operatorDashboard.coastOverride.get() ? NeutralModeValue.Coast : NeutralModeValue.Brake);
        }
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Superstructure/Hood/Goal", goal);

        // Turn off E-stop when homing
        if (goal == Goal.HOME) {
            motor.undoEmergencyStop(NeutralModeValue.Brake);
        }

        if (DriverStation.isDisabled() || motor.isEmergencyStopped() || goal == Goal.HOME_FINALIZE) {
            motor.setVoltageRequest(0.0);
        } else if (goal == Goal.HOME) {
            motor.setVoltageRequest(-0.5);
        } else {
            double setpointRad = goal.setpointRad.getAsDouble();
            boolean isInTrench = robotState.isInTrench(robotState.getTranslation().
                    plus(getMechanismTransform().getTranslation().toTranslation2d()));
            Logger.recordOutput("Superstructure/Hood/IsInTrench", isInTrench);
            if (isInTrench) {
                setpointRad = Math.min(setpointRad, maxPositionUnderTrench);
            }
            setpointRad = MathUtil.clamp(setpointRad, minPositionRad, maxPositionRad);
            if (BuildConstants.isSimOrReplay)
                Logger.recordOutput("Superstructure/Hood/SetpointRad", setpointRad);
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

    public boolean isEmergencyStopped() {
        return motor.isEmergencyStopped();
    }

    public boolean isDisconnected() {
        return !motor.isConnected();
    }

    public Transform3d getMechanismTransform() {
        return new Transform3d(
                new Translation3d(Units.inchesToMeters(-6.910046), Units.inchesToMeters(-9.109744), Units.inchesToMeters(12.861381)),
                new Rotation3d(0.0, Units.degreesToRadians(-90.0), Math.PI)
        ).plus(new Transform3d(
                new Translation3d(),
                new Rotation3d(0.0, motor.getPositionRad(), 0.0)
        ));
    }
}
