package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.Util;
import frc.lib.devices.motor.CtrlSparkMaxConfig;
import frc.lib.devices.motor.MechanismSim;
import frc.lib.devices.motor.Motor;
import frc.lib.network.LoggedTunablePIDF;
import frc.lib.subsystem.Periodic;
import frc.lib.wpilib.TrapezoidProfile;
import frc.robot.BuildConstants;
import frc.robot.Constants;
import frc.robot.OperatorDashboard;
import frc.robot.RobotState;
import frc.robot.shooting.ShootingKinematics;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import java.util.Objects;
import java.util.function.DoubleSupplier;

public class Turret implements Periodic {
    // 0 = shooting away from intake
    private static final double minPositionRad = Units.degreesToRadians(-92);
    private static final double maxPositionRad = Units.degreesToRadians(408);
    private static final double initialPositionRad = 0.0;
    private static final double positionPastLimitForEmergencyStopRad = Units.degreesToRadians(5);

    private static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(8, 18);

    private static final double homingToleranceRad = Units.degreesToRadians(45.0);

    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private static final RobotState robotState = RobotState.get();

    private final Motor motor = Motor
            .createSparkMax(
                    "Superstructure/Turret",
                    11,
                    new CtrlSparkMaxConfig()
                            .withInverted(true)
                            .withCurrentLimit(60)
                            .withGearRatio(3.0 * 3.0 * (68.0 / 12.0))
                            .withNeutralMode(NeutralModeValue.Brake),
                    initialPositionRad,
                    MechanismSim.arm(
                            0.1004362678,
                            Units.inchesToMeters(7.5),
                            minPositionRad,
                            maxPositionRad,
                            false
                    )
            )
            .withPositionGains(switch (BuildConstants.mode) {
                case REAL, REPLAY -> new LoggedTunablePIDF("Superstructure/Turret/PositionGains")
                        .withP(10)
                        .withD(0.1);
                case SIM -> new LoggedTunablePIDF("Superstructure/Turret/PositionGains")
                        .withP(10.0)
                        .withD(0.1);
            })
            .withVelocityGains(switch (BuildConstants.mode) {
                case REAL, REPLAY -> new LoggedTunablePIDF("Superstructure/Turret/VelocityGains")
                        .withP(0.1)
                        .withS(0.2, StaticFeedforwardSignValue.UseVelocitySign)
                        .withV(0.3)
                        .withA(0.005);
                case SIM -> new LoggedTunablePIDF("Superstructure/Turret/VelocityGains")
                        .withV(0.1);
            });

    @RequiredArgsConstructor
    public enum Goal {
        SHOOT(() -> ShootingKinematics.get().getShootingParameters().headingRad()),
        AIM_AT_CLOSEST_HUB(() -> 0.0),
        ;

        /** Should be constant for every loop cycle */
        private final DoubleSupplier fieldRelativeSetpointRad;
    }

    @Setter
    @Getter
    private Goal goal = Goal.AIM_AT_CLOSEST_HUB;

    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
    private TrapezoidProfile.State state = new TrapezoidProfile.State(initialPositionRad, 0.0);

    private final Debouncer emergencyStopDebouncer = new Debouncer(1.0, Debouncer.DebounceType.kRising);

    @Getter
    private boolean homed = false;
    private boolean verifyingHoming = false;
    private double observedMinRad = initialPositionRad;
    private double observedMaxRad = initialPositionRad;

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
        if (verifyingHoming) {
            updateHomingVerification();
        }

        boolean shouldEmergencyStop = emergencyStopDebouncer.calculate(motor.getStatorCurrentAmps() >= 50) ||
                (motor.getAppliedVolts() > 0 &&
                        motor.getPositionRad() > (maxPositionRad + positionPastLimitForEmergencyStopRad)) ||
                (motor.getAppliedVolts() < 0 &&
                        motor.getPositionRad() < (minPositionRad - positionPastLimitForEmergencyStopRad));
        if (!motor.isEmergencyStopped()) {
            if ((shouldEmergencyStop || operatorDashboard.turretEStop.get()) && !BuildConstants.isSim) {
                motor.emergencyStop(NeutralModeValue.Coast);
                operatorDashboard.turretEStop.set(true);
            }
        } else {
            if (!operatorDashboard.turretEStop.get()) {
                motor.undoEmergencyStop(NeutralModeValue.Brake);
                operatorDashboard.turretEStop.set(false);
            }
        }

        // Apply network inputs
        if (!motor.isEmergencyStopped() && operatorDashboard.coastOverride.hasChanged()) {
            motor.setNeutralMode(operatorDashboard.coastOverride.get() ? NeutralModeValue.Coast : NeutralModeValue.Brake);
        }

        Logger.recordOutput("Superstructure/Turret/FieldRelativePositionRad", getFieldRelativePositionRad());
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Superstructure/Turret/Goal", goal);
        if (DriverStation.isDisabled() || motor.isEmergencyStopped() || !homed) {
            motor.setVoltageRequest(0.0);

            // Reset state to current position
            state = new TrapezoidProfile.State(motor.getPositionRad(), 0.0);
        } else {
            // See the comments above the lookaheadState and goalState variables for why we calculate two profiles

            double fieldRelativeSetpointRad = goal.fieldRelativeSetpointRad.getAsDouble();
            Logger.recordOutput("Superstructure/Turret/FieldRelativeSetpointRad", fieldRelativeSetpointRad);

            double mechanismSetpointRad = resolveMechanismSetpointFromFieldRelativeSetpoint(fieldRelativeSetpointRad);
            mechanismSetpointRad = MathUtil.clamp(mechanismSetpointRad, minPositionRad, maxPositionRad);
            Logger.recordOutput("Superstructure/Turret/OriginalMechanismSetpointRad", mechanismSetpointRad);
            TrapezoidProfile.State wantedState = new TrapezoidProfile.State(mechanismSetpointRad, 0.0);

            state = profile.calculate(Constants.loopPeriod, state, wantedState);
            Logger.recordOutput("Superstructure/Turret/ProfileSetpointRad", state.position);
            Logger.recordOutput("Superstructure/Turret/ProfileSetpointRadPerSec", state.velocity);

            motor.setMotionProfileRequest(state.position, state.velocity);
        }
    }

    private double resolveMechanismSetpointFromFieldRelativeSetpoint(double fieldRelativeSetpointRad) {
        final double wantedMechanismSetpointRad = MathUtil.angleModulus(convertFieldRelativePositionToMechanismPosition(fieldRelativeSetpointRad));

        final double currentMechanismPosition = motor.getPositionRad();
        Double closestSetpoint = null;
        final int possibleRotations = (int) Math.ceil(Units.radiansToRotations(maxPositionRad - minPositionRad));
        for (int rotation = -possibleRotations; rotation <= possibleRotations; rotation++) {
            double possibleSetpoint = wantedMechanismSetpointRad + rotation * 2.0 * Math.PI;
            if (possibleSetpoint < minPositionRad || possibleSetpoint > maxPositionRad) {
                continue;
            }

            if (closestSetpoint == null ||
                    Math.abs(possibleSetpoint - currentMechanismPosition) < Math.abs(closestSetpoint - currentMechanismPosition)) {
                closestSetpoint = possibleSetpoint;
            }
        }

        return Objects.requireNonNullElse(closestSetpoint, currentMechanismPosition);
    }

    private static double convertMechanismPositionToFieldRelativePosition(double mechanismPositionRad) {
        // add 180° - see comment at top of class
        return mechanismPositionRad + Math.PI + robotState.getRotation().getRadians();
    }

    private static double convertFieldRelativePositionToMechanismPosition(double fieldRelativePositionRad) {
        // subtract 180° - see comment at top of class
        return fieldRelativePositionRad - Math.PI - robotState.getRotation().getRadians();
    }

    public double getFieldRelativePositionRad() {
        return convertMechanismPositionToFieldRelativePosition(motor.getPositionRad());
    }

    public double getRobotRelativeHeadingVelocityRadPerSec() {
        return motor.getVelocityRadPerSec();
    }

    public void home() {
        motor.setEncoderPosition(initialPositionRad);
        state = new TrapezoidProfile.State(initialPositionRad, 0.0);

        observedMinRad = initialPositionRad;
        observedMaxRad = initialPositionRad;
        verifyingHoming = true;
        homed = false;

        operatorDashboard.turretNotHomedAlert.set(true);
        operatorDashboard.turretHomingFailedAlert.set(false);
        operatorDashboard.turretEStop.set(true);
    }

    private void updateHomingVerification() {
        double positionRad = motor.getPositionRad();
        observedMinRad = Math.min(observedMinRad, positionRad);
        observedMaxRad = Math.max(observedMaxRad, positionRad);

        double sweptRad = observedMaxRad - observedMinRad;
        Logger.recordOutput("Superstructure/Turret/Homing/ObservedMinRad", observedMinRad);
        Logger.recordOutput("Superstructure/Turret/Homing/ObservedMaxRad", observedMaxRad);
        Logger.recordOutput("Superstructure/Turret/Homing/SweptRad", sweptRad);
        Logger.recordOutput("Superstructure/Turret/Homing/RemainingRad",
                Math.max(0.0, (maxPositionRad - minPositionRad) - homingToleranceRad - sweptRad));

        if (sweptRad < (maxPositionRad - minPositionRad) - homingToleranceRad) {
            return;
        }

        verifyingHoming = false;

        if (Math.abs(observedMinRad - minPositionRad) <= homingToleranceRad) {
            homed = true;
            operatorDashboard.turretNotHomedAlert.set(false);
            operatorDashboard.turretEStop.set(false);
        } else {
            operatorDashboard.turretHomingFailedAlert.set(true);
        }
    }

    public Transform3d getMechanismTransform() {
        return new Transform3d(
                new Translation3d(ShootingKinematics.turretRotationAxisTransform.getX(), ShootingKinematics.turretRotationAxisTransform.getY(), ShootingKinematics.bottomOfFrameRailsToFlywheelHeightMeters),
                new Rotation3d(0.0, 0.0, 0.0)
        ).plus(new Transform3d(
                new Translation3d(),
                new Rotation3d(0.0, 0.0, motor.getPositionRad())
        ));
    }
}
