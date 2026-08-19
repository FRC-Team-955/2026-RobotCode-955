package frc.robot.subsystems.superstructure.turret;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.EnergyLogger;
import frc.lib.Util;
import frc.lib.motor.MotorIO;
import frc.lib.motor.MotorIOInputsAutoLogged;
import frc.lib.motor.RequestType;
import frc.lib.subsystem.Periodic;
import frc.robot.BuildConstants;
import frc.robot.Constants;
import frc.robot.OperatorDashboard;
import frc.robot.RobotState;
import frc.robot.shooting.ShootingKinematics;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import java.util.OptionalDouble;

import static frc.robot.subsystems.superstructure.turret.TurretConstants.*;

/**
 * Azimuth turret that aims the shooter, so the drivebase doesn't have to. Always tracks, and
 * refuses to move until {@link #captureHomingPoint()} has been called at both hard stops.
 */
public class Turret implements Periodic {
    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private static final ShootingKinematics shootingKinematics = ShootingKinematics.get();
    private static final RobotState robotState = RobotState.get();
    private static final EnergyLogger energyLogger = EnergyLogger.get();

    private final MotorIO io = createIO();
    private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
    private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

    private Double homingAngleAtMaxRad = null;
    @Getter
    private boolean homed = false;

    private final Alert motorDisconnectedAlert = new Alert("Turret motor is disconnected.", Alert.AlertType.kError);
    public final Alert highTemperatureAlert = new Alert("Turret motor temperature is high.", Alert.AlertType.kWarning);
    private final Alert homingSpanAlert = new Alert("Turret homing points don't match the expected travel - check the gear ratio.", Alert.AlertType.kWarning);
    private final Alert outOfRangeAlert = new Alert("Turret can't reach the shot - the drivebase needs to turn.", Alert.AlertType.kWarning);

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

        if (BuildConstants.isSim) {
            homed = true;
            operatorDashboard.turretNotHomedAlert.set(false);
        }
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Superstructure/Turret", inputs);

        motorDisconnectedAlert.set(!inputs.connected);
        highTemperatureAlert.set(inputs.temperatureCelsius > 50);

        energyLogger.reportPowerUsage("Turret", inputs.connected ? inputs.appliedVolts * inputs.supplyCurrentAmps : 0.0);

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
        Logger.recordOutput("Superstructure/Turret/Homed", homed);

        if (DriverStation.isDisabled() || !homed) {
            io.setRequest(RequestType.VoltageVolts, 0.0);
            outOfRangeAlert.set(false);
            setpointState = new TrapezoidProfile.State(getPositionRad(), 0.0);
            return;
        }

        double wantedAngleRad = MathUtil.angleModulus(
                shootingKinematics.getShootingParameters().headingRad() - robotState.getRotation().getRadians()
        );
        Logger.recordOutput("Superstructure/Turret/WantedAngleRad", wantedAngleRad);

        OptionalDouble reachableRad = reachableSetpointRad(wantedAngleRad, getPositionRad());
        double setpointRad = reachableRad.orElse(MathUtil.clamp(wantedAngleRad, minPositionRad, maxPositionRad));
        outOfRangeAlert.set(reachableRad.isEmpty());

        setpointState = profile.calculate(
                Constants.loopPeriod,
                setpointState,
                new TrapezoidProfile.State(setpointRad, 0.0)
        );

        Logger.recordOutput("Superstructure/Turret/SetpointRad", setpointRad);
        Logger.recordOutput("Superstructure/Turret/ProfileSetpointRad", setpointState.position);
        Logger.recordOutput("Superstructure/Turret/ErrorRad", setpointRad - getPositionRad());

        io.setRequest(RequestType.PositionRad, setpointState.position);
    }

    public double getPositionRad() {
        return inputs.positionRad;
    }

    public double getVelocityRadPerSec() {
        return inputs.velocityRadPerSec;
    }

    /** Field relative direction the shooter is pointed */
    public double getHeadingRad() {
        return MathUtil.angleModulus(robotState.getRotation().getRadians() + getPositionRad());
    }

    /**
     * Records one end of the turret's travel. Call it with the turret held against the CCW (max)
     * stop, then again against the CW (min) stop, and the second call finishes homing.
     */
    public void captureHomingPoint() {
        if (homingAngleAtMaxRad == null) {
            homingAngleAtMaxRad = getPositionRad();
            return;
        }

        double spanErrorRad = homingSpanErrorRad(homingAngleAtMaxRad, getPositionRad());
        homingAngleAtMaxRad = null;

        io.setEncoderPosition(homingSeedPositionRad(spanErrorRad));
        homed = true;
        operatorDashboard.turretNotHomedAlert.set(false);
        homingSpanAlert.set(Math.abs(spanErrorRad) > homingSpanToleranceRad);
    }

    public boolean isDisconnected() {
        return !inputs.connected;
    }

    public Transform3d transform() {
        // PLACEHOLDER: rotates about the middle of the robot
        return new Transform3d(
                new Translation3d(0.0, 0.0, Units.inchesToMeters(12.861381)),
                new Rotation3d(0.0, 0.0, getPositionRad())
        );
    }
}
