package frc.robot.shooting;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import frc.lib.AllianceFlipUtil;
import frc.lib.Util;
import frc.lib.network.LoggedTunableNumber;
import frc.lib.subsystem.Periodic;
import frc.robot.*;
import frc.robot.subsystems.superstructure.flywheel.Flywheel;
import frc.robot.subsystems.superstructure.flywheel.FlywheelConstants;
import frc.robot.subsystems.superstructure.hood.Hood;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import java.util.OptionalDouble;
import java.util.function.DoubleFunction;

import static frc.robot.subsystems.drive.DriveConstants.driveConfig;

public class ShootingKinematics implements Periodic {
    // KEEP SYNCED WITH shooting_regression.py
    private static final double bottomOfFrameRailsToShooterHeightMeters = Units.inchesToMeters(12.861380);
    private static final double shooterRadiusToCenterOfBallExitMeters = Units.inchesToMeters(4.602756);

    private static final LoggedTunableNumber phaseDelay = new LoggedTunableNumber("ShootingKinematics/PhaseDelay", 0.03);
    private static final LoggedTunableNumber robotVelocityScalar = new LoggedTunableNumber("ShootingKinematics/RobotVelocityScalar (DEBUG ONLY)", 1);
    private static final LoggedTunableNumber yDistToleranceInches = new LoggedTunableNumber("ShootingKinematics/YDistToleranceInches", 3.0);
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

    private static final InterpolatingDoubleTreeMap velocityToRPM = new InterpolatingDoubleTreeMap();

    static {
        velocityToRPM.put(ShootingRegression.calculateVelocityMetersPerSec(1.5, 0.0), 1800.0);
        velocityToRPM.put(ShootingRegression.calculateVelocityMetersPerSec(1.75, 0.0), 1900.0);
        velocityToRPM.put(ShootingRegression.calculateVelocityMetersPerSec(2.0, 0.0), 2000.0);
        velocityToRPM.put(ShootingRegression.calculateVelocityMetersPerSec(2.25, 0.0), 2000.0);
        velocityToRPM.put(ShootingRegression.calculateVelocityMetersPerSec(2.5, 0.0), 2000.0);
        velocityToRPM.put(ShootingRegression.calculateVelocityMetersPerSec(2.75, 0.0), 2000.0);
        velocityToRPM.put(ShootingRegression.calculateVelocityMetersPerSec(3.0, 0.0), 2000.0);
        velocityToRPM.put(ShootingRegression.calculateVelocityMetersPerSec(3.25, 0.0), 2100.0);
        velocityToRPM.put(ShootingRegression.calculateVelocityMetersPerSec(3.5, 0.0), 2100.0);
        velocityToRPM.put(ShootingRegression.calculateVelocityMetersPerSec(3.75, 0.0), 2200.0);
        velocityToRPM.put(ShootingRegression.calculateVelocityMetersPerSec(4.0, 0.0), 2200.0);
        velocityToRPM.put(ShootingRegression.calculateVelocityMetersPerSec(4.25, 0.0), 2200.0);
        velocityToRPM.put(ShootingRegression.calculateVelocityMetersPerSec(4.5, 0.0), 2300.0);
        velocityToRPM.put(ShootingRegression.calculateVelocityMetersPerSec(4.75, 0.0), 2400.0);
        velocityToRPM.put(ShootingRegression.calculateVelocityMetersPerSec(5.0, 0.0), 2400.0);
        velocityToRPM.put(ShootingRegression.calculateVelocityMetersPerSec(5.25, 0.0), 2500.0);
    }

    private static final RobotState robotState = RobotState.get();
    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private static final HubShiftTracker hubShiftTracker = HubShiftTracker.get();

    private static final Flywheel flywheel = Flywheel.get();
    private static final Hood hood = Hood.get();

    @Getter
    private ShootingParameters shootingParameters = new ShootingParameters(0, 0, 0, OptionalDouble.empty(), false);
    @Getter
    private boolean shootingParametersMet = false;

    private final Alert noValidShootingParametersAlert = new Alert("Could not find valid shooting parameters.", Alert.AlertType.kInfo);

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
                shootingParameters.angleRad() + operatorDashboard.hoodSmudgeDegrees.get(),
                shootingParameters.headingRad(),
                shootingParameters.timeOfFlightSeconds(),
                shootingParameters.isPass()
        );

        Logger.recordOutput("ShootingKinematics/ShootingParameters/TimeOfFlightSeconds", shootingParameters.timeOfFlightSeconds().orElse(-1.0));
        Logger.recordOutput("ShootingKinematics/ShootingParameters/IsPass", shootingParameters.isPass());

        Logger.recordOutput("ShootingKinematics/ShootingParameters/HeadingRad", shootingParameters.headingRad());
        Logger.recordOutput("ShootingKinematics/ShootingParameters/HeadingRadMeasured", robotState.getPose().getRotation().getRadians());
        Logger.recordOutput("ShootingKinematics/ShootingParameters/VelocityRPM", shootingParameters.velocityRPM());
        Logger.recordOutput("ShootingKinematics/ShootingParameters/VelocityRPMMeasured", flywheel.getVelocityRPM());
        Logger.recordOutput("ShootingKinematics/ShootingParameters/AngleRad", shootingParameters.angleRad());
        Logger.recordOutput("ShootingKinematics/ShootingParameters/AngleRadMeasured", hood.getShotAngleRad());

        boolean shiftMet = shootingParameters.isPass() || operatorDashboard.disableShiftTracking.get() || hubShiftTracker.getShiftInfo().active();
        Logger.recordOutput("ShootingKinematics/ShiftMet", shiftMet);

        boolean yDistMet = Math.abs(getFuelExitToHub().transform.getY()) <= yDistToleranceInches.get();
        Logger.recordOutput("ShootingKinematics/YDistMet", yDistMet);

        boolean velocityMet = Math.abs(flywheel.getVelocityRPM() - shootingParameters.velocityRPM()) <= velocityToleranceRPM.get();
        Logger.recordOutput("ShootingKinematics/VelocityMet", velocityMet);

        boolean angleMet = Math.abs(hood.getShotAngleRad() - shootingParameters.angleRad()) <= Units.degreesToRadians(hoodToleranceDeg.get());
        Logger.recordOutput("ShootingKinematics/AngleMet", angleMet);

        shootingParametersMet = shiftMet && yDistMet && velocityMet && angleMet;
    }

    private static final LoggedTunableNumber shootHubManualFlywheelRPM = new LoggedTunableNumber("ShootingKinematics/ShootHubManual/FlywheelRPM", 2000.0);
    private static final LoggedTunableNumber shootHubManualAngleDegrees = new LoggedTunableNumber("ShootingKinematics/ShootHubManual/AngleDegrees", 70.0);

    private static final LoggedTunableNumber shootTowerManualFlywheelRPM = new LoggedTunableNumber("ShootingKinematics/ShootTowerManual/FlywheelRPM", 5.0);
    private static final LoggedTunableNumber shootTowerManualAngleDegrees = new LoggedTunableNumber("ShootingKinematics/ShootTowerManual/AngleDegrees", 45.0);

    private static final LoggedTunableNumber passManualFlywheelRPM = new LoggedTunableNumber("ShootingKinematics/PassManual/FlywheelRPM", 2000.0);
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
        if (robotState.getPose().getX() > AllianceFlipUtil.applyX(FieldConstants.LinesVertical.neutralZoneNear)) {
            return new ShootingParameters(
                    passManualFlywheelRPM.get(),
                    Units.degreesToRadians(passManualAngleDegrees.get()),
                    AllianceFlipUtil.shouldFlip() ? Math.PI : 0.0,
                    OptionalDouble.empty(),
                    true
            );
        }

        FuelExitToHub fuelExitToHub = getFuelExitToHub();

        double xyDist = fuelExitToHub.transform().getTranslation().toTranslation2d().getNorm();
        // Logger.recordOutput("ShootingKinematics/XYDist", xyDist);

        // 1. Compute velocity and angle from regression and rotate shooting vector into field coordinates
        // Note that using fuel exit pose instead of robot pose automatically takes care
        // of compensating for theta difference when looking from center of robot and from
        // fuel exit point
        ChassisSpeeds robotSpeeds = robotState.getMeasuredChassisSpeedsFieldRelative().times(robotVelocityScalar.get());
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
                fuelExitTranslation.apply(hood.getPositionRad()).toTranslation2d()
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
        //Logger.recordOutput("ShootingKinematics/Velocity", v);
        //Logger.recordOutput("ShootingKinematics/Phi", phi);
        //Logger.recordOutput("ShootingKinematics/Theta", theta);

        return new ShootingParameters(
                BuildConstants.mode == BuildConstants.Mode.SIM
                        ? Units.radiansPerSecondToRotationsPerMinute(v / FlywheelConstants.flywheelRadiusMeters)
                        : velocityToRPM.get(v),
                phi,
                theta,
                OptionalDouble.of(toF),
                false
        );
    }

    private FuelExitToHub getFuelExitToHub() {
        Pose2d robotPose2d = robotState.getPose()
                .exp(robotState.getMeasuredChassisSpeedsRobotRelative().toTwist2d(phaseDelay.get()));
        Pose3d fuelExitPose = new Pose3d(
                new Pose3d(robotPose2d)
                        .transformBy(new Transform3d(
                                fuelExitTranslation.apply(hood.getPositionRad()),
                                new Rotation3d()
                        ))
                        .getTranslation(),
                new Rotation3d(
                        robotPose2d.getRotation()
                                .plus(fuelExitRotation)
                )
        );
        Logger.recordOutput("ShootingKinematics/FuelExitPose", fuelExitPose);

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

    // Robot velocity centered at the shooter relative to the hub (positive x is towards hub, positive y is CLOCKWISE)
    // robotSpeeds field relative
    private Translation2d robotVelocityHubRelative(Translation2d robotSpeeds) {
        FuelExitToHub fuelExitToHub = getFuelExitToHub();
        return robotSpeeds.rotateBy(fuelExitRotation.minus(fuelExitToHub.angle()));
    }

    public double rotationAboutHubRadiansPerSec(Translation2d fieldRelativeMetersPerSec) {
        Translation2d hubRelative = robotVelocityHubRelative(fieldRelativeMetersPerSec);
        FuelExitToHub fuelExitToHub = getFuelExitToHub();

        // CW positive for hubRelative, so need to negate into CCW positive
        return -hubRelative.getY() / fuelExitToHub.transform.getTranslation().toTranslation2d().getNorm();
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
