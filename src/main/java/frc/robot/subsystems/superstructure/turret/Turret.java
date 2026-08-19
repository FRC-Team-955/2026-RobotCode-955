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
import frc.lib.motor.MotorIOInputsAutoLogged;
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
 * Azimuth turret that aims the shooter, so that the drivebase doesn't have to. It tracks the
 * target the whole time it's enabled - there's no reason to wait until we're shooting.
 * <p>
 * The heading {@link ShootingKinematics} hands out is the field relative heading the <i>robot</i>
 * would need to be at to make the shot, so the turret setpoint is just that heading minus the
 * heading we're actually at.
 * <p>
 * The NEO has no absolute encoder, so the turret is homed by hand off of both hard stops:
 * push it CCW into the max stop and press the home button, then push it CW into the min stop and
 * press it again. Two points instead of one because it also tells us how far the turret really
 * travelled, which is a check on {@link TurretConstants#gearRatio}. Until that's done the turret
 * doesn't know where it's aiming, so it isn't allowed to move.
 */
public class Turret implements Periodic {
    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private static final ShootingKinematics shootingKinematics = ShootingKinematics.get();
    private static final RobotState robotState = RobotState.get();
    private static final EnergyLogger energyLogger = EnergyLogger.get();

    private final TurretIO io = createIO();
    private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    // A turret carrying a shooter has real inertia, so the setpoint gets ramped instead of
    // stepped - a raw position request half a turn away just slams the motor into saturation
    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
    private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

    /** Encoder reading taken at the CCW (max) hard stop, null until the first homing press */
    private Double homingRawAtMaxRad = null;
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
            // Nobody is around to hand push the turret into the stops in sim
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
            io.setVoltageRequest(0.0);
            outOfRangeAlert.set(false);
            // Start from wherever we actually are, so enabling doesn't snap the turret
            setpointState = new TrapezoidProfile.State(getPositionRad(), 0.0);
            return;
        }

        // Robot relative angle we'd have to be at to make the shot
        double wantedRad = MathUtil.angleModulus(
                shootingKinematics.getShootingParameters().headingRad() - robotState.getRotation().getRadians()
        );
        Logger.recordOutput("Superstructure/Turret/WantedRad", wantedRad);

        // The turret can't cross its stops, so when the robot rotates the target past 180 this
        // hands back the position on the other side and the turret sweeps back around through zero
        OptionalDouble reachableRad = reachableSetpointRad(wantedRad, getPositionRad());
        double setpointRad = reachableRad.orElse(MathUtil.clamp(wantedRad, minPositionRad, maxPositionRad));
        // Nothing left to do about it if even the long way around is out of travel
        outOfRangeAlert.set(reachableRad.isEmpty());

        setpointState = profile.calculate(
                Constants.loopPeriod,
                setpointState,
                new TrapezoidProfile.State(setpointRad, 0.0)
        );

        Logger.recordOutput("Superstructure/Turret/SetpointRad", setpointRad);
        Logger.recordOutput("Superstructure/Turret/ProfileSetpointRad", setpointState.position);
        Logger.recordOutput("Superstructure/Turret/ErrorRad", setpointRad - getPositionRad());

        io.setPositionRequest(setpointState.position);
    }

    public double getPositionRad() {
        return inputs.positionRad;
    }

    public double getVelocityRadPerSec() {
        return inputs.velocityRadPerSec;
    }

    /**
     * Field relative direction the shooter is pointed, i.e. the same thing
     * {@link ShootingKinematics.ShootingParameters#headingRad()} asks for.
     */
    public double getHeadingRad() {
        return MathUtil.angleModulus(robotState.getRotation().getRadians() + getPositionRad());
    }

    /**
     * Records one end of the turret's travel. Call it with the turret held against the CCW (max)
     * stop, then again with it held against the CW (min) stop, and the second call finishes homing.
     */
    public void captureHomingPoint() {
        if (homingRawAtMaxRad == null) {
            homingRawAtMaxRad = getPositionRad();
            System.out.println("********** Turret homing: max captured **********");
            return;
        }

        // We're sitting at the min stop, so we know both where we are and how far the turret
        // actually travelled between the stops
        double spanErrorRad = homingSpanErrorRad(homingRawAtMaxRad, getPositionRad());
        homingRawAtMaxRad = null;

        io.setEncoderPosition(homingSeedPositionRad(spanErrorRad));
        homed = true;
        operatorDashboard.turretNotHomedAlert.set(false);

        homingSpanAlert.set(Math.abs(spanErrorRad) > homingSpanToleranceRad);
        System.out.printf(
                "********** Turret homed, span error %.2f degrees **********%n",
                Units.radiansToDegrees(spanErrorRad)
        );
    }

    public boolean isDisconnected() {
        return !inputs.connected;
    }

    public Transform3d transform() {
        // PLACEHOLDER: the turret rotates about the robot's center until we know where the
        // bearing actually sits
        return new Transform3d(
                new Translation3d(0.0, 0.0, Units.inchesToMeters(12.861381)),
                new Rotation3d(0.0, 0.0, getPositionRad())
        );
    }
}
