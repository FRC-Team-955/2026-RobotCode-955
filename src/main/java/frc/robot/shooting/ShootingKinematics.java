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
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.flywheel.FlywheelConstants;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import java.util.OptionalDouble;
import java.util.function.DoubleFunction;
import java.util.function.DoubleUnaryOperator;

import static frc.robot.subsystems.drive.DriveConstants.driveConfig;

public class ShootingKinematics implements Periodic {
    // KEEP SYNCED WITH shooting_regression.py
    private static final double bottomOfFrameRailsToShooterHeightMeters = Units.inchesToMeters(12.861380);
    private static final double shooterRadiusToCenterOfBallExitMeters = Units.inchesToMeters(4.602756);

    private static final LoggedTunableNumber phaseDelay = new LoggedTunableNumber("ShootingKinematics/PhaseDelay", 0.03);
    private static final LoggedTunableNumber robotVelocityScalar = new LoggedTunableNumber("ShootingKinematics/RobotVelocityScalar (DEBUG ONLY)", 1);
    private static final LoggedTunableNumber headingToleranceDeg = new LoggedTunableNumber("ShootingKinematics/HeadingToleranceDegrees", 10.0);
    private static final LoggedTunableNumber headingTolerancePassingDeg = new LoggedTunableNumber("ShootingKinematics/HeadingTolerancePassingDegrees", 20.0);
    private static final LoggedTunableNumber headingVelocityToleranceDegPerSec = new LoggedTunableNumber("ShootingKinematics/HeadingVelocityToleranceDegreesPerSec", 30.0);
    public static final LoggedTunableNumber velocityToleranceRPM = new LoggedTunableNumber("ShootingKinematics/VelocityToleranceRPM", 100);
    public static final LoggedTunableNumber hoodToleranceDeg = new LoggedTunableNumber("ShootingKinematics/HoodToleranceDegrees", 3.0);

    public static final DoubleFunction<Translation3d> fuelExitTranslation = (hoodAngleRad) -> new Translation3d(
            Units.inchesToMeters(-6.910046) + Math.cos(hoodAngleRad) * shooterRadiusToCenterOfBallExitMeters,
            Units.inchesToMeters(-9.172244),
            driveConfig.bottomOfFrameRailsToCenterOfWheelsMeters() +
                    driveConfig.wheelRadiusMeters() +
                    bottomOfFrameRailsToShooterHeightMeters +
                    Math.sin(hoodAngleRad) * shooterRadiusToCenterOfBallExitMeters
    );
    public static final Rotation2d fuelExitRotation = Rotation2d.k180deg;

    private static final DoubleUnaryOperator velocityToRPM;
    //private static final InterpolatingDoubleTreeMap velocityToRPMMap = new InterpolatingDoubleTreeMap();

    static {
        //velocityToRPMMap.put(6.8, 1850.0);
        //velocityToRPMMap.put(7.02, 1900.0);
        //velocityToRPMMap.put(7.22, 1950.0);
        //velocityToRPMMap.put(7.45, 2050.0);
        //velocityToRPMMap.put(7.64, 2100.0);
        //velocityToRPMMap.put(7.94, 2250.0);
        //velocityToRPMMap.put(8.26, 2350.0);
        //velocityToRPMMap.put(8.40, 2450.0);
        //velocityToRPMMap.put(9.10, 2650.0);
        //velocityToRPM = (x) -> velocityToRPMMap.get(x);

        // https://www.desmos.com/calculator/0ow99dd1u0
        velocityToRPM = (x) -> 366.65817 * x - 672.63778;
    }

    private static final RobotState robotState = RobotState.get();
    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private static final HubShiftTracker hubShiftTracker = HubShiftTracker.get();

    private static final Superstructure superstructure = Superstructure.get();

    @Getter
    private ShootingParameters shootingParameters = new ShootingParameters(0, 0, 0, OptionalDouble.empty(), false);
    @Getter
    private boolean shootingParametersMet = false;
    @Getter
    private boolean shiftMet = false;

    private final Debouncer velocityMetDebouncer = new Debouncer(0.15, Debouncer.DebounceType.kFalling);

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
        if (operatorDashboard.getSelectedScoringMode() == OperatorDashboard.ScoringMode.ShootAndPassAutomatic) {
            shootingParameters = getShootingParametersAutomatic();
        } else {
            shootingParameters = getShootingParametersManual();
        }
        shootingParameters = new ShootingParameters(
                shootingParameters.velocityRPM() + operatorDashboard.flywheelSmudgeRPM.get(),
                shootingParameters.angleRad() + Units.degreesToRadians(operatorDashboard.hoodSmudgeDegrees.get()),
                shootingParameters.headingRad(),
                shootingParameters.timeOfFlightSeconds(),
                shootingParameters.isPass()
        );

        //Logger.recordOutput("ShootingKinematics/ShootingParameters/TimeOfFlightSeconds", shootingParameters.timeOfFlightSeconds().orElse(-1.0));
        //Logger.recordOutput("ShootingKinematics/ShootingParameters/IsPass", shootingParameters.isPass());

        double headingVelocitySetpoint = rotationAboutHubRadiansPerSec(robotState.getMeasuredChassisSpeedsFieldRelative());
        double headingVelocityMeasurement = robotState.getMeasuredChassisSpeedsFieldRelative().omegaRadiansPerSecond;

        //Logger.recordOutput("ShootingKinematics/ShootingParameters/HeadingRad", shootingParameters.headingRad());
        //Logger.recordOutput("ShootingKinematics/ShootingParameters/HeadingRadMeasured", robotState.getPose().getRotation().getRadians());
        //Logger.recordOutput("ShootingKinematics/ShootingParameters/HeadingVelocityRadPerSec", headingVelocitySetpoint);
        //Logger.recordOutput("ShootingKinematics/ShootingParameters/HeadingVelocityRadPerSecMeasured", headingVelocityMeasurement);
        //Logger.recordOutput("ShootingKinematics/ShootingParameters/VelocityRPM", shootingParameters.velocityRPM());
        //Logger.recordOutput("ShootingKinematics/ShootingParameters/VelocityRPMMeasured", superstructure.flywheel.getVelocityRPM());
        //Logger.recordOutput("ShootingKinematics/ShootingParameters/AngleRad", shootingParameters.angleRad());
        //Logger.recordOutput("ShootingKinematics/ShootingParameters/AngleRadMeasured", superstructure.hood.getShotAngleRad());

        shiftMet = shootingParameters.isPass() || operatorDashboard.disableShiftTracking.get() || hubShiftTracker.getShiftInfo().active();
        Logger.recordOutput("ShootingKinematics/ShiftMet", shiftMet);

        boolean headingMet = operatorDashboard.manualAiming.get() ||
                Math.abs(
                        MathUtil.angleModulus(robotState.getPose().getRotation().getRadians())
                                - MathUtil.angleModulus(shootingParameters.headingRad())
                ) <= Units.degreesToRadians(
                        shootingParameters.isPass()
                                ? headingTolerancePassingDeg.get()
                                : headingToleranceDeg.get()
                );
        Logger.recordOutput("ShootingKinematics/HeadingMet", headingMet);

        boolean headingVelocityMet = operatorDashboard.manualAiming.get() ||
                Math.abs(headingVelocityMeasurement - headingVelocitySetpoint)
                        <= Units.degreesToRadians(headingVelocityToleranceDegPerSec.get());
        Logger.recordOutput("ShootingKinematics/HeadingVelocityMet", headingVelocityMet);

        boolean velocityMet = velocityMetDebouncer.calculate(
                Math.abs(superstructure.flywheel.getVelocityRPM() - shootingParameters.velocityRPM())
                        <= velocityToleranceRPM.get()
        );
        Logger.recordOutput("ShootingKinematics/VelocityMet", velocityMet);

        boolean angleMet = Math.abs(superstructure.hood.getShotAngleRad() - shootingParameters.angleRad())
                <= Units.degreesToRadians(hoodToleranceDeg.get());
        Logger.recordOutput("ShootingKinematics/AngleMet", angleMet);

        shootingParametersMet = shiftMet && headingMet && headingVelocityMet && velocityMet && angleMet;
        Logger.recordOutput("ShootingKinematics/ShootingParametersMet", shootingParametersMet);
    }

    private static final LoggedTunableNumber shootHubManualFlywheelRPM = new LoggedTunableNumber("ShootingKinematics/ShootHubManual/FlywheelRPM", 2000.0);
    private static final LoggedTunableNumber shootHubManualAngleDegrees = new LoggedTunableNumber("ShootingKinematics/ShootHubManual/AngleDegrees", 70.0);

    private static final LoggedTunableNumber shootTowerManualFlywheelRPM = new LoggedTunableNumber("ShootingKinematics/ShootTowerManual/FlywheelRPM", 2000.0);
    private static final LoggedTunableNumber shootTowerManualAngleDegrees = new LoggedTunableNumber("ShootingKinematics/ShootTowerManual/AngleDegrees", 62.0);

    private static final LoggedTunableNumber passManualFlywheelRPM = new LoggedTunableNumber("ShootingKinematics/PassManual/FlywheelRPM", 2400.0);
    private static final LoggedTunableNumber passManualAngleDegrees = new LoggedTunableNumber("ShootingKinematics/PassManual/AngleDegrees", 45.0);

    private ShootingParameters getShootingParametersManual() {
        double headingRad = getFuelExitToHub().angle().getRadians();
        return switch (operatorDashboard.getSelectedScoringMode()) {
            case ShootHubManual -> new ShootingParameters(
                    shootHubManualFlywheelRPM.get(),
                    Units.degreesToRadians(shootHubManualAngleDegrees.get()),
                    headingRad,
                    OptionalDouble.empty(),
                    false
            );
            case ShootTowerManual -> new ShootingParameters(
                    shootTowerManualFlywheelRPM.get(),
                    Units.degreesToRadians(shootTowerManualAngleDegrees.get()),
                    headingRad,
                    OptionalDouble.empty(),
                    false
            );
            case PassManual -> new ShootingParameters(
                    passManualFlywheelRPM.get(),
                    Units.degreesToRadians(passManualAngleDegrees.get()),
                    headingRad,
                    OptionalDouble.empty(),
                    true
            );

            // something went wrong
            case ShootAndPassAutomatic -> new ShootingParameters(
                    0.0,
                    0.0,
                    robotState.getRotation().getRadians(),
                    OptionalDouble.empty(),
                    false
            );
        };
    }

    private ShootingParameters getShootingParametersAutomatic() {
        ChassisSpeeds robotSpeeds = robotState.getMeasuredChassisSpeedsFieldRelative().times(robotVelocityScalar.get());

        if (
                AllianceFlipUtil.shouldFlip()
                        ? robotState.getPose().getX() < FieldConstants.LinesVertical.neutralZoneFar
                        : robotState.getPose().getX() > FieldConstants.LinesVertical.neutralZoneNear
        ) {
            double targetX = AllianceFlipUtil.applyX(1.5);
            double targetY = robotState.getPose().getY() > FieldConstants.LinesHorizontal.center
                    ? 6.0
                    : 2.0;

            double heading = new Translation2d(targetX, targetY)
                    .minus(getFuelExitPose(robotState.getPose()).getTranslation().toTranslation2d())
                    .getAngle()
                    .plus(fuelExitRotation)
                    .getRadians();
            return new ShootingParameters(
                    2100.0 + 250.0 * Math.max(0.0, AllianceFlipUtil.applyY(robotState.getTranslation().getX()) - 6.0),
                    Units.degreesToRadians(passManualAngleDegrees.get()),
                    0.15 * (AllianceFlipUtil.shouldFlip() ? -1 : 1) * robotSpeeds.vyMetersPerSecond + heading,
                    OptionalDouble.empty(),
                    true
            );
        }

        FuelExitToHub fuelExitToHub = getFuelExitToHub();

        double xyDist = fuelExitToHub.transform().getTranslation().toTranslation2d().getNorm();
        //Logger.recordOutput("ShootingKinematics/XYDist", xyDist);

        // 1. Compute velocity and angle from regression and rotate shooting vector into field coordinates
        // Note that using fuel exit pose instead of robot pose automatically takes care
        // of compensating for theta difference when looking from center of robot and from
        // fuel exit point
        Translation2d robotSpeedsFuelExitRelative = robotVelocityHubRelative(new Translation2d(
                robotSpeeds.vxMetersPerSecond,
                robotSpeeds.vyMetersPerSecond
        ));
        //Logger.recordOutput("ShootingKinematics/RobotSpeedsRotated", robotSpeedsFuelExitRelative);

        double v0 = ShootingRegression.calculateVelocityMetersPerSec(xyDist, robotSpeedsFuelExitRelative.getX());
        double angle = ShootingRegression.calculateAngleRad(xyDist, robotSpeedsFuelExitRelative.getX());
        double toF = ShootingRegression.calculateToFSeconds(xyDist, robotSpeedsFuelExitRelative.getX());

        double vx2d = v0 * Math.cos(angle);
        double vz = v0 * Math.sin(angle);

        // Logger.recordOutput("ShootingKinematics/ShotSpeedTargetFieldRelative", v0);
        Translation2d robotShotFieldRelative = new Translation2d(vx2d, fuelExitToHub.angle());

        // Logger.recordOutput("ShootingKinematics/ShotTargetFieldRelative", robotShotFieldRelative);

        // 2. Now subtract tangential robot velocity from initial shooting vector to get final
        // shooting vector
        // Note that we must subtract the fuel exit rotation to account for the robot speeds
        // being fuel exit relative
        Translation2d tangentialRobotVelocityRobotRelative = new Translation2d(0, robotSpeedsFuelExitRelative.rotateBy(fuelExitRotation.unaryMinus()).getY());
        robotShotFieldRelative = robotShotFieldRelative.plus(tangentialRobotVelocityRobotRelative.rotateBy(fuelExitToHub.angle));

        // Logger.recordOutput("ShootingKinematics/TangentialRobotVelocityFieldRelative", tangentialRobotVelocityRobotRelative.rotateBy(fuelExitToHub.angle));


        // 3. Account for drivebase angular velocity
        Vector<N3> fuelExitFieldRelative = new Translation3d(
                fuelExitTranslation.apply(superstructure.hood.getPositionRad()).toTranslation2d()
                        .rotateBy(robotState.getRotation())
        ).toVector();
        Vector<N3> angularVelocityVector = VecBuilder.fill(0.0, 0.0, robotSpeeds.omegaRadiansPerSecond);
        // ω⃗ × e⃗, where ω⃗ is angular velocity vector and e⃗ is exit vector
        Vector<N3> linearVelocityDueToAngularVelocity = Vector.cross(angularVelocityVector, fuelExitFieldRelative);
        Translation2d angularVelToLinearVel = new Translation2d(linearVelocityDueToAngularVelocity.get(0), linearVelocityDueToAngularVelocity.get(1));
        robotShotFieldRelative = robotShotFieldRelative.minus(angularVelToLinearVel);
        double vx = robotShotFieldRelative.getX();
        double vy = robotShotFieldRelative.getY();

        // 4. Now calculate phi, theta, and shooting magnitude from 3d shooting vector
        double v = Math.sqrt(vx * vx + vy * vy + vz * vz);
        double phi = Math.asin(vz / v);
        double theta = Math.atan2(vy, vx);
        //Logger.recordOutput("ShootingKinematics/ShootingParameters/VelocityMetersPerSec", shootingVelocity);
        //Logger.recordOutput("ShootingKinematics/Phi", phi);
        //Logger.recordOutput("ShootingKinematics/Theta", theta);

        return new ShootingParameters(
                BuildConstants.mode == BuildConstants.Mode.SIM
                        ? Units.radiansPerSecondToRotationsPerMinute(v / FlywheelConstants.flywheelRadiusMeters)
                        : velocityToRPM.applyAsDouble(v),
                phi,
                theta,
                OptionalDouble.of(toF),
                false
        );
    }

    private Pose3d getFuelExitPose(Pose2d robotPose2d) {
        return new Pose3d(
                new Pose3d(robotPose2d)
                        .transformBy(new Transform3d(
                                fuelExitTranslation.apply(superstructure.hood.getPositionRad()),
                                new Rotation3d()
                        ))
                        .getTranslation(),
                new Rotation3d(
                        robotPose2d.getRotation()
                                .plus(fuelExitRotation)
                )
        );
    }

    private FuelExitToHub getFuelExitToHub() {
        Pose2d robotPose2d = robotState.getPose()
                .exp(robotState.getMeasuredChassisSpeedsRobotRelative().toTwist2d(phaseDelay.get()));
        Pose3d fuelExitPose = getFuelExitPose(robotPose2d);
        //Logger.recordOutput("ShootingKinematics/FuelExitPose", fuelExitPose);

        Translation3d hubTranslation = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint);
        Pose3d hubPose = new Pose3d(hubTranslation, new Rotation3d());
        return new FuelExitToHub(
                new Transform3d(fuelExitPose, hubPose),
                hubPose.getTranslation().toTranslation2d()
                        .minus(fuelExitPose.getTranslation().toTranslation2d())
                        .getAngle()
                        .plus(fuelExitRotation)
        );
    }

    /**
     * Robot velocity centered at the shooter relative to the hub (positive x is towards hub, positive y is CLOCKWISE)
     * robotSpeeds field relative
     */
    private Translation2d robotVelocityHubRelative(Translation2d robotSpeeds) {
        FuelExitToHub fuelExitToHub = getFuelExitToHub();
        return robotSpeeds.rotateBy(fuelExitRotation.minus(fuelExitToHub.angle()));
    }

    // Rotation around hub from velocity, can add to drive rotation for aiming feedforward
    public double rotationAboutHubRadiansPerSec(Translation2d fieldRelativeMetersPerSec) {
        if (shootingParameters.isPass()) {
            return 0.0;
        }

        Translation2d hubRelative = robotVelocityHubRelative(fieldRelativeMetersPerSec);
        FuelExitToHub fuelExitToHub = getFuelExitToHub();

        // CW positive for hubRelative, so need to negate into CCW positive
        // tangential velocity in m/s / radius of circle = rotation about circle rad/sec
        return -hubRelative.getY() / fuelExitToHub.transform.getTranslation().toTranslation2d().getNorm();
    }
    /*
    // Estimated rotation due to tangential acceleration, add to drive for aiming feedforward
    public double rotationFeedforwardAcceleration(Translation2d fieldRelativeMetersPerSecSquared, Translation2d robotSpeeds) {
        Translation2d shootingParameters2dHubRelative = robotVelocityHubRelative(new Translation2d(shootingVelocity, shootingParameters.headingRad()));
        Translation2d robotVelocityHubRelative = robotVelocityHubRelative(robotSpeeds);
        // Again, positive Y is CLOCKWISE
        double tangentialAcceleration = robotVelocityHubRelative(fieldRelativeMetersPerSecSquared).getY();
    }
     */

    public double rotationAboutHubRadiansPerSec(ChassisSpeeds fieldRelativeSpeeds) {
        return rotationAboutHubRadiansPerSec(new Translation2d(
                fieldRelativeSpeeds.vxMetersPerSecond,
                fieldRelativeSpeeds.vyMetersPerSecond
        ));
    }

    public Translation2d getCenterOfRotationForAiming() {
        return fuelExitTranslation.apply(superstructure.hood.getPositionRad()).toTranslation2d();
    }

    private record FuelExitToHub(Transform3d transform, Rotation2d angle) {}

    public record ShootingParameters(
            double velocityRPM,
            double angleRad,
            double headingRad,
            OptionalDouble timeOfFlightSeconds,
            boolean isPass
    ) {}
}
