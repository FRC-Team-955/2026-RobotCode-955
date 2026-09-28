package frc.robot.shooting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.lib.AllianceFlipUtil;
import frc.lib.Util;
import frc.lib.network.LoggedTunableNumber;
import frc.lib.subsystem.Periodic;
import frc.robot.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Flywheel;
import frc.robot.subsystems.superstructure.Hood;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Turret;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.With;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleUnaryOperator;
import java.util.function.Supplier;

import static frc.robot.subsystems.drive.DriveConstants.carpetToBottomOfFrameRail;

public class ShootingKinematics implements Periodic {
    /** Actual RPM * slip constant = exerted RPM (linear speed of ball = surface speed) */
    public static final double slipConstant = 0.67;

    // KEEP SYNCED WITH shooting_regression.py
    public static final double bottomOfFrameRailsToFlywheelHeightMeters = Units.inchesToMeters(15.236467);
    private static final double centerOfFlywheelToCenterOfBallExitMeters = Units.inchesToMeters(4.602756);

    public static final Supplier<Transform2d> turretRotationAxisTransform = () -> new Transform2d(
            new Translation2d(
                    Units.inchesToMeters(-3.785046),
                    Units.inchesToMeters(-6.672244)
            ),
            Rotation2d.fromRadians(Turret.get().getRobotRelativePositionRad())
    );
    public static final double centerOfTurretRotationAxisToCenterOfFlywheelMeters = Units.inchesToMeters(3.026799);
    private static final Supplier<Transform3d> turretRotationAxisToFuelExitTransform = () -> new Transform3d(
            new Translation3d(
                    centerOfTurretRotationAxisToCenterOfFlywheelMeters - Math.cos(Hood.get().getPositionRad()) * centerOfFlywheelToCenterOfBallExitMeters,
                    0.0,
                    carpetToBottomOfFrameRail +
                            bottomOfFrameRailsToFlywheelHeightMeters +
                            Math.sin(Hood.get().getPositionRad()) * centerOfFlywheelToCenterOfBallExitMeters
            ),
            new Rotation3d()
    );

    private static final RobotState robotState = RobotState.get();
    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private static final HubShiftTracker hubShiftTracker = HubShiftTracker.get();
    private static final Superstructure superstructure = Superstructure.get();
    private static final Drive drive = Drive.get();

    private static final LoggedTunableNumber headingToleranceDeg = new LoggedTunableNumber("ShootingKinematics/HeadingToleranceDegrees", 10.0);
    private static final LoggedTunableNumber headingTolerancePassingDeg = new LoggedTunableNumber("ShootingKinematics/HeadingTolerancePassingDegrees", 20.0);
    private static final LoggedTunableNumber headingVelocityToleranceDegPerSec = new LoggedTunableNumber("ShootingKinematics/HeadingVelocityToleranceDegreesPerSec", 30.0);
    public static final LoggedTunableNumber velocityToleranceRPM = new LoggedTunableNumber("ShootingKinematics/VelocityToleranceRPM", 100);
    public static final LoggedTunableNumber hoodToleranceDeg = new LoggedTunableNumber("ShootingKinematics/HoodToleranceDegrees", 3.0);

    private static final DoubleUnaryOperator passVelocityToRPM = (x) -> 316 * x - 456 + 100;

    @Getter
    private ShootingParameters shootingParameters = new ShootingParameters(0, 0, 0, 0, 0, false);
    @Getter
    private double lastScoringTimeOfFlightSeconds = 0.0;
    @Getter
    private boolean shootingParametersMet = false;
    @Getter
    private boolean shiftMet = false;

    private final Debouncer velocityMetDebouncer = new Debouncer(0.15, Debouncer.DebounceType.kFalling);
    private final Debouncer headingVelocityDebouncer = new Debouncer(0.10, Debouncer.DebounceType.kFalling);
    private final Debouncer orientationDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kFalling);

    private ShootingParameters noPhaseDelayParameters = new ShootingParameters(
            0.0,
            0.0,
            0.0,
            robotState.getRotation().getRadians(),
            0.0,
            false
    );

    private static ShootingKinematics instance;

    public static synchronized ShootingKinematics get() {
        if (instance == null) {
            instance = new ShootingKinematics();
        }

        return instance;
    }

    private ShootingKinematics() {
        if (instance != null) {
            Util.error("Duplicate ShootingKinematics created");
        }
    }

    @Override
    public void periodicBeforeCommands() {
        Logger.recordOutput("ShootingKinematics/FuelExitPose", new Pose3d(robotState.getPose()).transformBy(getFuelExitTransform()));

        if (operatorDashboard.getSelectedScoringMode() == OperatorDashboard.ScoringMode.ShootAndPassAutomatic) {
            var shooterParams = getShootingParametersAutomaticForPhaseDelay(PhaseDelay.Shooter);
            var turretParams = getShootingParametersAutomaticForPhaseDelay(PhaseDelay.Turret);
            shootingParameters = new ShootingParameters(
                    shooterParams.velocityRPM(),
                    shooterParams.velocityXYMetersPerSec(),
                    shooterParams.angleRad(),
                    turretParams.headingRad(),
                    turretParams.headingVelocityRadPerSec(),
                    shooterParams.isPass()
            );
            noPhaseDelayParameters = getShootingParametersAutomaticForPhaseDelay(PhaseDelay.None);
        } else {
            shootingParameters = getShootingParametersManual();
            shootingParameters = shootingParameters.withVelocityRPM(
                    shootingParameters.velocityRPM() + operatorDashboard.manualFlywheelRPMSmudge.get()
            );
            noPhaseDelayParameters = shootingParameters;
        }

        if (BuildConstants.isSimOrReplay) {
            Logger.recordOutput("ShootingKinematics/ShootingParameters/None/LastScoringTimeOfFlightSeconds", lastScoringTimeOfFlightSeconds);
            Logger.recordOutput("ShootingKinematics/ShootingParameters/None/IsPass", noPhaseDelayParameters.isPass());
        }

        if (BuildConstants.isSimOrReplay) {
            Logger.recordOutput("ShootingKinematics/ShootingParameters/None/HeadingRad", noPhaseDelayParameters.headingRad());
            Logger.recordOutput("ShootingKinematics/ShootingParameters/Turret/HeadingRad", shootingParameters.headingRad());
            Logger.recordOutput("ShootingKinematics/ShootingParameters/HeadingRadMeasured", superstructure.turret.getFieldRelativePositionRad());

            Logger.recordOutput("ShootingKinematics/ShootingParameters/None/HeadingVelocityRadPerSec", noPhaseDelayParameters.headingVelocityRadPerSec());
            Logger.recordOutput("ShootingKinematics/ShootingParameters/Turret/HeadingVelocityRadPerSec", shootingParameters.headingVelocityRadPerSec());
            Logger.recordOutput("ShootingKinematics/ShootingParameters/HeadingVelocityRadPerSecMeasured", superstructure.turret.getHeadingVelocityRadPerSec());

            Logger.recordOutput("ShootingKinematics/ShootingParameters/None/AngleRad", noPhaseDelayParameters.angleRad());
            Logger.recordOutput("ShootingKinematics/ShootingParameters/Shooter/AngleRad", shootingParameters.angleRad());
            Logger.recordOutput("ShootingKinematics/ShootingParameters/AngleRadMeasured", superstructure.hood.getShotAngleRad());

            Logger.recordOutput("ShootingKinematics/ShootingParameters/Shooter/VelocityRPM", shootingParameters.velocityRPM());
            Logger.recordOutput("ShootingKinematics/ShootingParameters/VelocityRPMMeasured", superstructure.flywheel.getVelocityRPM());
        }
        Logger.recordOutput("ShootingKinematics/ShootingParameters/None/VelocityRPM", noPhaseDelayParameters.velocityRPM());

        shiftMet = operatorDashboard.disableShiftTracking.get() || hubShiftTracker.getShiftInfo().active();
        Logger.recordOutput("ShootingKinematics/ShiftMet", shiftMet);

        boolean headingMet = operatorDashboard.manualAiming.get() ||
                Math.abs(
                        MathUtil.angleModulus(superstructure.turret.getFieldRelativePositionRad() - noPhaseDelayParameters.headingRad())
                ) <= Units.degreesToRadians(
                        noPhaseDelayParameters.isPass()
                                ? headingTolerancePassingDeg.get()
                                : headingToleranceDeg.get()
                );
        Logger.recordOutput("ShootingKinematics/HeadingMet", headingMet);

        boolean headingVelocityMet = operatorDashboard.manualAiming.get() ||
                Math.abs(superstructure.turret.getHeadingVelocityRadPerSec() - noPhaseDelayParameters.headingVelocityRadPerSec())
                        <= Units.degreesToRadians(headingVelocityToleranceDegPerSec.get());
        Logger.recordOutput("ShootingKinematics/HeadingVelocityMet", headingVelocityMet);
        Logger.recordOutput("ShootingKinematics/HeadingVelocityDelta", superstructure.turret.getHeadingVelocityRadPerSec() - noPhaseDelayParameters.headingVelocityRadPerSec());
        headingVelocityMet = headingVelocityDebouncer.calculate(headingVelocityMet);
        if (BuildConstants.isSimOrReplay)
            Logger.recordOutput("ShootingKinematics/HeadingVelocityMetDebounced", headingVelocityMet);

        boolean velocityMet = Math.abs(superstructure.flywheel.getVelocityRPM() - noPhaseDelayParameters.velocityRPM())
                <= velocityToleranceRPM.get();
        Logger.recordOutput("ShootingKinematics/VelocityMet", velocityMet);
        velocityMet = velocityMetDebouncer.calculate(velocityMet);
        if (BuildConstants.isSimOrReplay) Logger.recordOutput("ShootingKinematics/VelocityMetDebounced", velocityMet);

        boolean angleMet = Math.abs(superstructure.hood.getShotAngleRad() - noPhaseDelayParameters.angleRad())
                <= Units.degreesToRadians(hoodToleranceDeg.get());
        Logger.recordOutput("ShootingKinematics/AngleMet", angleMet);

        boolean uncertaintyMet = robotState.getPoseUncertaintyLinearMeters() < 0.3 &&
                robotState.getPoseUncertaintyAngularRad() < 0.005;
        Logger.recordOutput("ShootingKinematics/UncertaintyMet", uncertaintyMet);

        boolean orientationMet = !drive.isPitchedOrRolled();
        Logger.recordOutput("ShootingKinematics/OrientationMet", orientationMet);
        orientationMet = orientationDebouncer.calculate(orientationMet);
        if (BuildConstants.isSimOrReplay)
            Logger.recordOutput("ShootingKinematics/OrientationMetDebounced", orientationMet);

        shootingParametersMet = noPhaseDelayParameters.isPass()
                ? headingMet && headingVelocityMet
                : shiftMet && headingMet && headingVelocityMet && velocityMet && angleMet && uncertaintyMet /*&& orientationMet*/;
        Logger.recordOutput("ShootingKinematics/ShootingParametersMet", shootingParametersMet);
    }

    private static final LoggedTunableNumber shootHubManualFlywheelRPM = new LoggedTunableNumber("ShootingKinematics/ShootHubManual/FlywheelRPM", 2000.0);
    private static final LoggedTunableNumber shootHubManualAngleDegrees = new LoggedTunableNumber("ShootingKinematics/ShootHubManual/AngleDegrees", 70.0);

    private static final LoggedTunableNumber shootTowerManualFlywheelRPM = new LoggedTunableNumber("ShootingKinematics/ShootTowerManual/FlywheelRPM", 2000.0);
    private static final LoggedTunableNumber shootTowerManualAngleDegrees = new LoggedTunableNumber("ShootingKinematics/ShootTowerManual/AngleDegrees", 62.0);

    private static final LoggedTunableNumber passManualFlywheelRPM = new LoggedTunableNumber("ShootingKinematics/PassManual/FlywheelRPM", 2400.0);
    private static final LoggedTunableNumber passManualAngleDegrees = new LoggedTunableNumber("ShootingKinematics/PassManual/AngleDegrees", Units.radiansToDegrees(Hood.convertBetweenShotAngleAndHoodAngleRad(Hood.maxPositionRad)));

    private ShootingParameters getShootingParametersManual() {
        double headingRad = getFuelExitToTarget(PhaseDelay.None).angle().getRadians();
        return switch (operatorDashboard.getSelectedScoringMode()) {
            case ShootHubManual -> new ShootingParameters(
                    shootHubManualFlywheelRPM.get(),
                    0.0, // if we are using this we have bigger issues than acceleration compensation
                    Units.degreesToRadians(shootHubManualAngleDegrees.get()),
                    headingRad,
                    0.0,
                    false
            );
            case ShootTowerManual -> new ShootingParameters(
                    shootTowerManualFlywheelRPM.get(),
                    0.0, // again, bigger issues
                    Units.degreesToRadians(shootTowerManualAngleDegrees.get()),
                    headingRad,
                    0.0,
                    false
            );
            case PassManual -> new ShootingParameters(
                    passManualFlywheelRPM.get(),
                    0.0,
                    Units.degreesToRadians(passManualAngleDegrees.get()),
                    headingRad,
                    0.0,
                    true
            );

            // something went wrong
            case ShootAndPassAutomatic -> new ShootingParameters(
                    0.0,
                    0.0,
                    0.0,
                    superstructure.turret.getFieldRelativePositionRad(),
                    0.0,
                    false
            );
        };
    }

    private ShootingParameters getShootingParametersAutomaticForPhaseDelay(PhaseDelay phaseDelay) {
        String key = "ShootingKinematics/ShootingParameters/" + phaseDelay.name() + "/";

        ChassisSpeeds robotSpeeds = robotState.getMeasuredChassisSpeedsFieldRelative();

        FuelExitToTarget fuelExitToTarget = getFuelExitToTarget(phaseDelay);
        Logger.recordOutput(key + "FuelExitToTarget", fuelExitToTarget);

        double xyDist = fuelExitToTarget.translation().toTranslation2d().getNorm();
        Logger.recordOutput(key + "XYDist", xyDist);

        // 1. Compute velocity and angle from regression and rotate shooting vector into field coordinates
        // Note that using fuel exit pose instead of robot pose automatically takes care
        // of compensating for theta difference when looking from center of robot and from
        // fuel exit point
        Translation2d robotSpeedsTargetRelative = getRobotVelocityTargetRelative(new Translation2d(
                robotSpeeds.vxMetersPerSecond,
                robotSpeeds.vyMetersPerSecond
        ), phaseDelay);
        if (BuildConstants.isSimOrReplay)
            Logger.recordOutput(key + "RobotSpeedsRotated", robotSpeedsTargetRelative);

        double v0;
        double angle;
        boolean isPass = shouldPass();
        if (isPass) {
            v0 = PassingRegression.calculateVelocityMetersPerSec(xyDist, robotSpeedsTargetRelative.getX());
            angle = PassingRegression.angleRad;
        } else {
            v0 = ScoringRegression.calculateVelocityMetersPerSec(xyDist, robotSpeedsTargetRelative.getX());
            angle = ScoringRegression.calculateAngleRad(xyDist, robotSpeedsTargetRelative.getX());
            if (phaseDelay == PhaseDelay.None) {
                lastScoringTimeOfFlightSeconds = ScoringRegression.calculateToFSeconds(xyDist, robotSpeedsTargetRelative.getX());
            }
        }

        if (BuildConstants.isSimOrReplay)
            Logger.recordOutput(key + "InitialShotVelocity", v0);

        double vx2d = v0 * Math.cos(angle);
        double vz = v0 * Math.sin(angle);

        Translation2d shotVelFieldRelative = new Translation2d(vx2d, fuelExitToTarget.angle());
        if (BuildConstants.isSimOrReplay)
            Logger.recordOutput(key + "InitialShotHorizontalVelocityFieldRelative", shotVelFieldRelative);

        // 2. Now subtract tangential robot velocity from initial shooting vector to get final
        // shooting vector
        // Note that we must subtract the angle to the target to account for the robot speeds
        // being target relative
        Translation2d tangentialRobotVelocityFieldRelative = new Translation2d(0, robotSpeedsTargetRelative.getY())
                .rotateBy(fuelExitToTarget.angle());
        if (BuildConstants.isSimOrReplay)
            Logger.recordOutput(key + "TangentialRobotVelocityFieldRelative", tangentialRobotVelocityFieldRelative);
        shotVelFieldRelative = shotVelFieldRelative.minus(tangentialRobotVelocityFieldRelative);

        // 3. Account for drivebase angular velocity
        Vector<N3> fuelExitFieldRelative = new Translation3d(
                getFuelExitTransform().getTranslation().toTranslation2d()
                        .rotateBy(robotState.getRotation())
        ).toVector();
        Vector<N3> angularVelocityVector = VecBuilder.fill(0.0, 0.0, robotSpeeds.omegaRadiansPerSecond);
        // ω⃗ × e⃗, where ω⃗ is angular velocity vector and e⃗ is exit vector
        Vector<N3> linearVelocityDueToAngularVelocity = Vector.cross(angularVelocityVector, fuelExitFieldRelative);
        Translation2d angularVelToLinearVel = new Translation2d(linearVelocityDueToAngularVelocity.get(0), linearVelocityDueToAngularVelocity.get(1));
        shotVelFieldRelative = shotVelFieldRelative.minus(angularVelToLinearVel);

        // Extract adjusted horizontal components
        if (BuildConstants.isSimOrReplay)
            Logger.recordOutput(key + "FinalShotHorizontalVelocityFieldRelative", shotVelFieldRelative);
        double vx = shotVelFieldRelative.getX();
        double vy = shotVelFieldRelative.getY();

        // 4. Now calculate phi, theta, and shooting magnitude from 3d shooting vector
        double v = Math.sqrt(vx * vx + vy * vy + vz * vz);
        double phi = Math.asin(vz / v);
        double theta = Math.atan2(vy, vx);
        Logger.recordOutput(key + "VelocityMetersPerSec", v);
        if (BuildConstants.isSimOrReplay) {
            Logger.recordOutput(key + "Phi", phi);
            Logger.recordOutput(key + "Theta", theta);
        }

        double velocityRPM = Units.radiansPerSecondToRotationsPerMinute(v / Flywheel.radiusMeters);
        return new ShootingParameters(
                BuildConstants.isSim
                        ? velocityRPM
                        : (
                        isPass
                                ? passVelocityToRPM.applyAsDouble(v) + operatorDashboard.manualFlywheelRPMSmudge.get()
                                : velocityRPM / (slipConstant + operatorDashboard.slipConstantSmudge.get())
                ),
                Math.sqrt(vx * vx + vy * vy),
                phi,
                theta,
                getTotalHeadingFeedforward(
                        drive.getConstrainer().getWantedLinearSpeed(),
                        drive.getConstrainer().getFieldRelativeAccelerationLinear(),
                        phaseDelay
                ),
                isPass
        );
    }

    private boolean shouldPass() {
        return AllianceFlipUtil.shouldFlip()
                ? robotState.getPose().getX() < FieldConstants.LinesVertical.neutralZoneFar
                : robotState.getPose().getX() > FieldConstants.LinesVertical.neutralZoneNear;
    }

    private Translation3d getTarget() {
        if (shouldPass()) {
            double targetX = AllianceFlipUtil.applyX(1.5);
            double targetY = robotState.getPose().getY() > FieldConstants.LinesHorizontal.center
                    ? 6.0
                    : 2.0;

            return new Translation3d(targetX, targetY, 0.0);
        }

        return AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint);
    }

    private FuelExitToTarget getFuelExitToTarget(PhaseDelay phaseDelay) {
        Pose2d robotPose2d = robotState.getPose()
                .exp(robotState.getMeasuredChassisSpeedsRobotRelative().toTwist2d(phaseDelay.getValue()));
        Pose3d fuelExitPose = new Pose3d(robotPose2d).transformBy(getFuelExitTransform());

        Pose3d hubPose = new Pose3d(getTarget(), new Rotation3d());
        return new FuelExitToTarget(
                new Transform3d(fuelExitPose, hubPose).getTranslation(),
                hubPose.getTranslation().toTranslation2d()
                        .minus(fuelExitPose.getTranslation().toTranslation2d())
                        .getAngle()
        );
    }

    /**
     * Robot velocity centered at the shooter relative to the hub (positive x is towards hub, positive y is CLOCKWISE)
     * robotSpeeds field relative
     */
    private Translation2d getRobotVelocityTargetRelative(Translation2d robotSpeeds, PhaseDelay phaseDelay) {
        FuelExitToTarget fuelExitToTarget = getFuelExitToTarget(phaseDelay);
        return robotSpeeds.rotateBy(fuelExitToTarget.angle().unaryMinus());
    }

    /** Rotation around hub from velocity, can add to drive rotation for aiming feedforward */
    private double getRotationAboutTargetRadiansPerSecForTurret(Translation2d fieldRelativeMetersPerSec, PhaseDelay phaseDelay) {
        Translation2d targetRelative = getRobotVelocityTargetRelative(fieldRelativeMetersPerSec, phaseDelay);
        FuelExitToTarget fuelExitToTarget = getFuelExitToTarget(phaseDelay);

        // CW positive for hubRelative, so need to negate into CCW positive
        // tangential velocity in m/s / radius of circle = rotation about circle rad/sec
        return -targetRelative.getY() / fuelExitToTarget.translation().toTranslation2d().getNorm();
    }

    /**
     * Estimated rotation due to tangential acceleration, add to drive for aiming feedforward.
     * Returns rotation in rad/sec
     */
    private double getHeadingFeedforwardDueToAcceleration(Translation2d fieldRelativeMetersPerSecSquared) {
        //Translation2d shootingParameters2dHubRelative = robotVelocityTargetRelativeForDrivebase(new Translation2d(shootingVelocityXY, shootingParameters.headingRad()));
        //Translation2d robotVelocityHubRelative = robotVelocityTargetRelativeForDrivebase(robotSpeeds);
        // Again, positive Y is CLOCKWISE
        //Translation2d tangentialAccelerationHubRelative = robotVelocityTargetRelativeForDrivebase(fieldRelativeMetersPerSecSquared);
        double tangentialAccelerationShotRelative = fieldRelativeMetersPerSecSquared.rotateBy(new Rotation2d(-shootingParameters.headingRad())).getY();
        return tangentialAccelerationShotRelative / noPhaseDelayParameters.velocityXYMetersPerSec;
    }

    private double getTotalHeadingFeedforward(Translation2d fieldRelativeSpeeds, Translation2d fieldRelativeMetersPerSecSquared, PhaseDelay phaseDelay) {
        return getRotationAboutTargetRadiansPerSecForTurret(fieldRelativeSpeeds, phaseDelay) + getHeadingFeedforwardDueToAcceleration(fieldRelativeMetersPerSecSquared);
    }

    public Transform3d getFuelExitTransform() {
        return new Transform3d(turretRotationAxisTransform.get())
                .plus(turretRotationAxisToFuelExitTransform.get());
    }

    private record FuelExitToTarget(Translation3d translation, Rotation2d angle) {}

    @With
    public record ShootingParameters(
            double velocityRPM,
            double velocityXYMetersPerSec,
            double angleRad,
            double headingRad,
            double headingVelocityRadPerSec,
            boolean isPass
    ) {}

    @RequiredArgsConstructor
    private enum PhaseDelay {
        Shooter(new LoggedTunableNumber("ShootingKinematics/ShooterPhaseDelay", 0.15)),
        Turret(new LoggedTunableNumber("ShootingKinematics/TurretPhaseDelay", 0.03)),
        None(null),
        ;

        private final LoggedTunableNumber value;

        private double getValue() {
            if (value == null) {
                return 0.0;
            } else {
                return value.get();
            }
        }
    }
}
