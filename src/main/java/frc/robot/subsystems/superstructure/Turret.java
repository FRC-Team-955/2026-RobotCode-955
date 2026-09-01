package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.Util;
import frc.lib.devices.motor.CtrlSparkMaxConfig;
import frc.lib.devices.motor.MechanismSim;
import frc.lib.devices.motor.Motor;
import frc.lib.network.LoggedTunablePIDF;
import frc.lib.subsystem.Periodic;
import frc.robot.BuildConstants;
import frc.robot.Constants;
import frc.robot.OperatorDashboard;
import frc.robot.RobotState;
import frc.robot.shooting.ShootingKinematics;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class Turret implements Periodic {
    // 0 = shooting away from intake
    private static final double minPositionRad = Units.degreesToRadians(-92);
    private static final double maxPositionRad = Units.degreesToRadians(408);
    private static final double initialPositionRad = 0.0;
    private static final double positionPastLimitForEmergencyStopRad = Units.degreesToRadians(5);

    private static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(5, 12);

    private static final double limitMarginRad = Units.degreesToRadians(20.0);

    private static final double unwindHysteresisRad = Units.degreesToRadians(15.0);

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
                        .withP(1.0);
            })
            .withVelocityGains(switch (BuildConstants.mode) {
                case REAL, REPLAY -> new LoggedTunablePIDF("Superstructure/Turret/VelocityGains")
                        .withP(0.1)
                        .withS(0.2, StaticFeedforwardSignValue.UseVelocitySign)
                        .withV(0.3)
                        .withA(0.005);
                case SIM -> new LoggedTunablePIDF("Superstructure/Turret/VelocityGains")
                        .withV(1.0);
            });

    @RequiredArgsConstructor
    public enum Goal {
        SHOOT(() -> ShootingKinematics.get().getShootingParameters().headingRad()),
        AIM_AT_CLOSEST_HUB(() -> ShootingKinematics.get().getShootingParameters().headingRad()),
        ;

        /** Should be constant for every loop cycle */
        private final DoubleSupplier fieldHeadingRad;
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

            double fieldHeadingRad = goal.fieldHeadingRad.getAsDouble();
            double setpointRad = resolveSetpointRad(fieldHeadingRad);
            setpointRad = MathUtil.clamp(setpointRad, minPositionRad, maxPositionRad);
            if (BuildConstants.isSimOrReplay) {
                Logger.recordOutput("Superstructure/Turret/WantedFieldHeadingRad", fieldHeadingRad);
                Logger.recordOutput("Superstructure/Turret/OriginalSetpointRad", setpointRad);
            }
            TrapezoidProfile.State wantedState = new TrapezoidProfile.State(setpointRad, 0.0);

            state = profile.calculate(Constants.loopPeriod, state, wantedState);
            Logger.recordOutput("Superstructure/Turret/ProfileSetpointRad", state.position);

            motor.setMotionProfileRequest(state.position, state.velocity);
        }
    }

    private double resolveSetpointRad(double fieldHeadingRad) {
        double wantedRad = fieldHeadingRad - robotState.getRotation().getRadians() - Math.PI;

        double currentRad = state.position;

        double nearestRad = Double.NaN;
        double roomiestRad = Double.NaN;
        int firstWrap = (int) Math.ceil((minPositionRad - wantedRad) / (2.0 * Math.PI));
        for (int wrap = firstWrap; ; wrap++) {
            double candidateRad = wantedRad + 2.0 * Math.PI * wrap;
            if (candidateRad > maxPositionRad) {
                break;
            }

            if (Double.isNaN(nearestRad) || Math.abs(candidateRad - currentRad) < Math.abs(nearestRad - currentRad)) {
                nearestRad = candidateRad;
            }
            if (Double.isNaN(roomiestRad) || headroomRad(candidateRad) > headroomRad(roomiestRad)) {
                roomiestRad = candidateRad;
            }
        }

        boolean nearingLimit = headroomRad(nearestRad) < limitMarginRad;
        boolean mayUnwind = nearingLimit || goal != Goal.SHOOT;
        boolean worthUnwinding = headroomRad(roomiestRad) > headroomRad(nearestRad) + unwindHysteresisRad;

        if (BuildConstants.isSimOrReplay) {
            Logger.recordOutput("Superstructure/Turret/NearingLimit", nearingLimit);
            Logger.recordOutput("Superstructure/Turret/Unwinding", mayUnwind && worthUnwinding);
        }

        return mayUnwind && worthUnwinding ? roomiestRad : nearestRad;
    }

    private static double headroomRad(double positionRad) {
        return Math.min(positionRad - minPositionRad, maxPositionRad - positionRad);
    }

    public double getRobotRelativeHeadingRad() {
        // add 180° - see comment at top of class
        return motor.getPositionRad() + Math.PI;
    }

    public double getFieldRelativeHeadingRad() {
        return getRobotRelativeHeadingRad() + robotState.getRotation().getRadians();
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
                new Translation3d(Units.inchesToMeters(10.0), 0.0, Units.inchesToMeters(6.25)),
                new Rotation3d(0.0, Units.degreesToRadians(90.0), 0.0)).plus(new Transform3d(
                new Translation3d(),
                new Rotation3d(0.0, -motor.getPositionRad(), 0.0)
        ));
    }
}
