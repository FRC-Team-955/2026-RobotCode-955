package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.Util;
import frc.lib.devices.motor.CtrlSparkMaxConfig;
import frc.lib.devices.motor.MechanismSim;
import frc.lib.devices.motor.Motor;
import frc.lib.network.LoggedTunablePIDF;
import frc.lib.subsystem.Periodic;
import frc.robot.*;
import frc.robot.shooting.ShootingKinematics;
import frc.robot.subsystems.drive.Drive;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class Turret implements Periodic {
    // 0 = shooting toward intake
    private static final double minPositionRad = Units.degreesToRadians(-160.0);
    private static final double maxPositionRad = Units.degreesToRadians(260.0);
    private static final double initialPositionRad = 0.0;

    private static final double positionPastLimitForEmergencyStopRad = Units.degreesToRadians(10);
    private static final double positionBeforeLimitToStopVelocityFeedforwardRad = Units.degreesToRadians(15);
    private static final double closeToWrappingRad = Units.degreesToRadians(90.0);
    private static final double homingToleranceRad = Units.degreesToRadians(15.0);

    private static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(12, 36);

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
                        .withP(5)
                        .withD(0.1);
                case SIM -> new LoggedTunablePIDF("Superstructure/Turret/PositionGains")
                        .withP(10.0)
                        .withD(0.1);
            })
            .withVelocityGains(switch (BuildConstants.mode) {
                case REAL, REPLAY -> new LoggedTunablePIDF("Superstructure/Turret/VelocityGains")
                        .withP(0.05)
                        .withS(0.3, StaticFeedforwardSignValue.UseVelocitySign)
                        .withV(0.4)
                        .withA(0.005);
                case SIM -> new LoggedTunablePIDF("Superstructure/Turret/VelocityGains")
                        .withP(0.1)
                        .withV(1.1);
            });

    @RequiredArgsConstructor
    public enum Goal {
        SHOOT(() -> ShootingKinematics.get().getShootingParameters().headingRad(), () -> ShootingKinematics.get().getShootingParameters().headingVelocityRadPerSec()),
        SHOOT_DEBUG(() -> RobotState.get().getRotation().getRadians() + Math.PI / 4.0, () -> 0.0),
        AIM_AT_CLOSEST_HUB(Turret::getFieldRelativeHeadingToClosestHubRad, () -> 0.0),
        ;

        /** Should be constant for every loop cycle */
        private final DoubleSupplier fieldRelativePositionSetpointRad;
        private final DoubleSupplier velocitySetpointRadPerSec;
    }

    @Setter
    @Getter
    private Goal goal = Goal.AIM_AT_CLOSEST_HUB;

    private final TimeInterpolatableBuffer<Double> motorPositionRadBuffer = TimeInterpolatableBuffer.createDoubleBuffer(5);

    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
    private TrapezoidProfile.State state = new TrapezoidProfile.State(initialPositionRad, 0.0);

    private final Debouncer emergencyStopDebouncer = new Debouncer(1.0, Debouncer.DebounceType.kRising);

    @Getter
    private boolean homed = false;
    private boolean needsToVerifyRange = false;
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
    public void updateAndProcessInputs() {
        motorPositionRadBuffer.addSample(Timer.getTimestamp(), motor.getPositionRad());
    }

    @Override
    public void periodicBeforeCommands() {
        if (needsToVerifyRange) {
            updateHomingVerification();
        }

        boolean shouldEmergencyStop = !homed ||
                (!DriverStation.isFMSAttached() && needsToVerifyRange) ||
                emergencyStopDebouncer.calculate(motor.getStatorCurrentAmps() >= 50) ||
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

            double fieldRelativePositionSetpointRad = goal.fieldRelativePositionSetpointRad.getAsDouble();
            Logger.recordOutput("Superstructure/Turret/FieldRelativePositionSetpointRad", fieldRelativePositionSetpointRad);

            double mechanismSetpointRad = resolveMechanismSetpointFromFieldRelativeSetpoint(fieldRelativePositionSetpointRad);
            mechanismSetpointRad = MathUtil.clamp(mechanismSetpointRad, minPositionRad, maxPositionRad);
            Logger.recordOutput("Superstructure/Turret/OriginalMechanismSetpointRad", mechanismSetpointRad);

            double velocitySetpointRadPerSec = goal.velocitySetpointRadPerSec.getAsDouble();
            velocitySetpointRadPerSec -= Drive.get().getConstrainer().getWantedAngularSpeed();
            Logger.recordOutput("Superstructure/Turret/VelocitySetpointRadPerSec", velocitySetpointRadPerSec);

            TrapezoidProfile.State wantedState = new TrapezoidProfile.State(mechanismSetpointRad, velocitySetpointRadPerSec);

            state = profile.calculate(Constants.loopPeriod, state, wantedState);
            Logger.recordOutput("Superstructure/Turret/ProfileSetpointRad", state.position);
            Logger.recordOutput("Superstructure/Turret/ProfileSetpointRadPerSec", state.velocity);

            double velocitySetpoint = state.velocity;
            boolean setVelocitySetpointToZero = (velocitySetpoint < 0 && motor.getPositionRad() < minPositionRad + positionBeforeLimitToStopVelocityFeedforwardRad) ||
                    (velocitySetpoint > 0 && motor.getPositionRad() > maxPositionRad - positionBeforeLimitToStopVelocityFeedforwardRad);
            if (setVelocitySetpointToZero) {
                velocitySetpoint = 0;
            }
            Logger.recordOutput("Superstructure/Turret/SetVelocitySetpointToZero", setVelocitySetpointToZero);

            motor.setMotionProfileRequest(state.position, velocitySetpoint);
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
        return mechanismPositionRad + robotState.getRotation().getRadians();
    }

    private static double convertFieldRelativePositionToMechanismPosition(double fieldRelativePositionRad) {
        return fieldRelativePositionRad - robotState.getRotation().getRadians();
    }

    public double getRobotRelativePositionRad() {
        return motor.getPositionRad();
    }

    public Optional<Double> getRobotRelativePositionRadAtTime(double timestampSeconds) {
        return motorPositionRadBuffer.getSample(timestampSeconds);
    }

    public double getFieldRelativePositionRad() {
        return convertMechanismPositionToFieldRelativePosition(motor.getPositionRad());
    }

    private static double getFieldRelativeHeadingToClosestHubRad() {
        Translation2d rotationAxis = robotState.getPose()
                .transformBy(ShootingKinematics.turretRotationAxisTransformAtCurrentTime.get())
                .getTranslation();
        Translation2d closestHub = rotationAxis.nearest(List.of(
                FieldConstants.Hub.topCenterPoint.toTranslation2d(),
                FieldConstants.Hub.oppTopCenterPoint.toTranslation2d()
        ));
        return closestHub.minus(rotationAxis).getAngle().getRadians();
    }

    public double getHeadingVelocityRadPerSec() {
        return motor.getVelocityRadPerSec();
    }

    public boolean isCloseToWrapping() {
        return !motor.isEmergencyStopped() &&
                (motor.getPositionRad() <= minPositionRad + closeToWrappingRad ||
                        motor.getPositionRad() >= maxPositionRad - closeToWrappingRad);
    }

    public void home() {
        motor.setEncoderPosition(initialPositionRad);
        state = new TrapezoidProfile.State(initialPositionRad, 0.0);

        observedMinRad = initialPositionRad;
        observedMaxRad = initialPositionRad;

        homed = true;
        operatorDashboard.turretNotHomedAlert.set(false);

        needsToVerifyRange = true;
        operatorDashboard.turretVerifyingAlert.set(true);
    }

    private void updateHomingVerification() {
        observedMinRad = Math.min(observedMinRad, motor.getPositionRad());
        observedMaxRad = Math.max(observedMaxRad, motor.getPositionRad());

        Logger.recordOutput("Superstructure/Turret/Homing/ObservedMinRad", observedMinRad);
        Logger.recordOutput("Superstructure/Turret/Homing/ObservedMaxRad", observedMaxRad);

        if (Math.abs(observedMinRad - minPositionRad) > homingToleranceRad ||
                Math.abs(observedMaxRad - maxPositionRad) > homingToleranceRad) {
            //double theoreticalRange = maxPositionRad - minPositionRad;
            //double observedRange = observedMaxRad - observedMinRad;
            //if (Math.abs(observedRange - theoreticalRange) < 2.0 * homingToleranceRad) {
            //    // Verification failed
            //    homed = false;
            //    operatorDashboard.turretNotHomedAlert.set(true);
            //
            //    needsToVerifyRange = false;
            //    operatorDashboard.turretVerifyingAlert.set(false);
            //}

            return;
        }

        needsToVerifyRange = false;
        operatorDashboard.turretVerifyingAlert.set(false);
    }

    public Transform3d getMechanismTransform() {
        Transform2d rotationAxis = ShootingKinematics.turretRotationAxisTransformAtCurrentTime.get();
        double mechanismTransformHeight = ShootingKinematics.bottomOfFrameRailsToFlywheelHeightMeters - Units.inchesToMeters(4.106366);
        return new Transform3d(
                new Translation3d(rotationAxis.getX(), rotationAxis.getY(), mechanismTransformHeight),
                new Rotation3d(rotationAxis.getRotation())
        );
    }
}