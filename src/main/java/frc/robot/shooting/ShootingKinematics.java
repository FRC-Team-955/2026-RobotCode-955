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

import java.util.function.DoubleFunction;

import static frc.robot.subsystems.drive.DriveConstants.driveConfig;

public class ShootingKinematics implements Periodic {
    // KEEP SYNCED WITH shooting_regression.py
    private static final double bottomOfFrameRailsToShooterHeightMeters = Units.inchesToMeters(12.861380);
    private static final double shooterRadiusToCenterOfBallExitMeters = Units.inchesToMeters(4.602756);

    private static final LoggedTunableNumber robotVelocityScalar = new LoggedTunableNumber("ShootingKinematics/RobotVelocityScalar", 1.2);
    private static final LoggedTunableNumber headingToleranceDeg = new LoggedTunableNumber("ShootingKinematics/HeadingToleranceDegrees", 5.0);
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
    private ShootingParameters shootingParameters = new ShootingParameters(0, 0, 0);
    @Getter
    private boolean validShootingParameters = false;
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
        validShootingParameters = operatorDashboard.getSelectedScoringMode() == OperatorDashboard.ScoringMode.ShootAndPassAutomatic
                ? updateShootingParametersAutomatic()
                : updateShootingParametersManual();
        shootingParameters = new ShootingParameters(
                shootingParameters.velocityRPM() + operatorDashboard.flywheelSmudgeRPM.get(),
                shootingParameters.hoodAngleRad() + operatorDashboard.hoodSmudgeDegrees.get(),
                shootingParameters.headingRad
        );
        Logger.recordOutput("ShootingKinematics/ShootingParameters", shootingParameters);
        Logger.recordOutput("ShootingKinematics/ValidShootingParameters", validShootingParameters);
        noValidShootingParametersAlert.set(!validShootingParameters);

        shootingParametersMet = validShootingParameters &&
                (operatorDashboard.manualAiming.get() || Math.abs(robotState.getPose().getRotation().getRadians() - shootingParameters.headingRad()) <= Units.degreesToRadians(headingToleranceDeg.get())) &&
                Math.abs(flywheel.getVelocityRPM() - shootingParameters.velocityRPM()) <= velocityToleranceRPM.get() &&
                Math.abs(hood.getPositionRad() - shootingParameters.hoodAngleRad()) <= Units.degreesToRadians(hoodToleranceDeg.get()) &&
                (operatorDashboard.disableShiftTracking.get() || hubShiftTracker.getShiftInfo().active());
        Logger.recordOutput("ShootingKinematics/ShootingParametersMet", shootingParametersMet);
    }

    private static final LoggedTunableNumber shootHubManualFlywheelRPM = new LoggedTunableNumber("ShootingKinematics/ShootHubManual/FlywheelRPM", 2000.0);
    private static final LoggedTunableNumber shootHubManualHoodDegrees = new LoggedTunableNumber("ShootingKinematics/ShootHubManual/HoodDegrees", 20.0);

    private static final LoggedTunableNumber shootTowerManualFlywheelRPM = new LoggedTunableNumber("ShootingKinematics/ShootTowerManual/FlywheelRPM", 5.0);
    private static final LoggedTunableNumber shootTowerManualHoodDegrees = new LoggedTunableNumber("ShootingKinematics/ShootTowerManual/HoodDegrees", 45.0);

    private static final LoggedTunableNumber passManualFlywheelRPM = new LoggedTunableNumber("ShootingKinematics/PassManual/FlywheelRPM", 5.0);
    private static final LoggedTunableNumber passManualHoodDegrees = new LoggedTunableNumber("ShootingKinematics/PassManual/HoodDegrees", 45.0);

    private boolean updateShootingParametersManual() {
        double headingRad = getFuelExitToHub().angle().getRadians();
        switch (operatorDashboard.getSelectedScoringMode()) {
            case ShootAndPassAutomatic -> {
                return false;
            }
            case ShootHubManual -> shootingParameters = new ShootingParameters(
                    shootHubManualFlywheelRPM.get(),
                    Units.degreesToRadians(shootHubManualHoodDegrees.get()),
                    headingRad
            );
            case ShootTowerManual -> shootingParameters = new ShootingParameters(
                    shootTowerManualFlywheelRPM.get(),
                    Units.degreesToRadians(shootTowerManualHoodDegrees.get()),
                    headingRad
            );
            case PassManual -> shootingParameters = new ShootingParameters(
                    passManualFlywheelRPM.get(),
                    Units.degreesToRadians(passManualHoodDegrees.get()),
                    headingRad
            );
        }

        return true;
    }

    private boolean isValidHoodAngle(double hoodAngleRad) {
        return hoodAngleRad > Units.degreesToRadians(15) && hoodAngleRad < Units.degreesToRadians(45);
    }

    private boolean updateShootingParametersAutomatic() {
        FuelExitToHub fuelExitToHub = getFuelExitToHub();

        double xyDist = fuelExitToHub.transform().getTranslation().toTranslation2d().getNorm();
        Logger.recordOutput("ShootingKinematics/XYDist", xyDist);

        // 1. Compute velocity and angle from regression
        ChassisSpeeds robotSpeeds = robotState.getMeasuredChassisSpeeds().times(robotVelocityScalar.get());
        Translation2d robotSpeedsFuelExitRelative = new Translation2d(
                robotSpeeds.vxMetersPerSecond,
                robotSpeeds.vyMetersPerSecond
        ).rotateBy(fuelExitRotation.plus(fuelExitToHub.angle().unaryMinus()));
        Logger.recordOutput("ShootingKinematics/RobotSpeedsRotated", robotSpeedsFuelExitRelative);

        double v0 = ShootingRegression.calculateVelocityMetersPerSec(xyDist, robotSpeedsFuelExitRelative.getX());
        double angle = ShootingRegression.calculateHoodAngleRad(xyDist, robotSpeedsFuelExitRelative.getX());

        double vx = v0 * Math.cos(angle);
        double vz = v0 * Math.sin(angle);

        // 2. Next, rotate shooting vector into field coordinates
        // Note that using fuel exit pose instead of robot pose automatically takes care
        // of compensating for theta difference when looking from center of robot and from
        // fuel exit point
        // We could use Translation2d to rotate it, but since vy = 0, it's simple enough
        // to just use trig
        double vy = vx * fuelExitToHub.angle().getSin();
        vx = vx * fuelExitToHub.angle().getCos();

        // 3. Now subtract tangential robot velocity from initial shooting vectory to get final
        // shooting vector
        // Note that we must subtract the fuel exit rotation to account for the robot speeds
        // being fuel exit relative
        vy -= robotSpeedsFuelExitRelative.rotateBy(fuelExitRotation.unaryMinus()).getY();

        // 4. Account for drivebase angular velocity
        Vector<N3> fuelExitFieldRelative = new Translation3d(
                fuelExitTranslation.apply(hood.getPositionRad()).toTranslation2d()
                        .rotateBy(robotState.getRotation())
        ).toVector();
        Vector<N3> angularVelocityVector = VecBuilder.fill(0.0, 0.0, robotSpeeds.omegaRadiansPerSecond);
        // ω⃗ × e⃗, where ω⃗ is angular velocity vector and e⃗ is exit vector
        Vector<N3> linearVelocityDueToAngularVelocity = Vector.cross(angularVelocityVector, fuelExitFieldRelative);
        vx -= linearVelocityDueToAngularVelocity.get(0);
        vy -= linearVelocityDueToAngularVelocity.get(1);

        // 5. Log final shooting vector
//        Logger.recordOutput("ShootingKinematics/Velocity/X", vx);
//        Logger.recordOutput("ShootingKinematics/Velocity/Y", vy);
//        Logger.recordOutput("ShootingKinematics/Velocity/Z", vz);

        // 6. Now calculate phi, theta, and shooting magnitude from 3d shooting vector
        double v = Math.sqrt(vx * vx + vy * vy + vz * vz);
        double phi = Math.asin(vz / v);
        double theta = Math.atan2(vy, vx);
//        Logger.recordOutput("ShootingKinematics/Velocity", v);
//        Logger.recordOutput("ShootingKinematics/Phi", phi);
//        Logger.recordOutput("ShootingKinematics/Theta", theta);

        shootingParameters = new ShootingParameters(
                BuildConstants.mode == BuildConstants.Mode.SIM
                        ? Units.radiansPerSecondToRotationsPerMinute(v / FlywheelConstants.flywheelRadiusMeters)
                        : velocityToRPM.get(v),
                phi,
                theta
        );
        return true;
    }

    private FuelExitToHub getFuelExitToHub() {
        Pose2d robotPose2d = robotState.getPose();
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

    public double rotationAboutHubRadiansPerSec(Translation2d fieldRelativeMetersPerSec) {
        Translation2d fuelExitToHubPerp = getFuelExitToHub().transform.getTranslation().toTranslation2d()
                .rotateBy(new Rotation2d(- Math.PI / 2));

        // Dot product projection of the robot velocity onto the perpendicular direction to the hub
        double tangentialVelocity = fuelExitToHubPerp.dot(fieldRelativeMetersPerSec) / fuelExitToHubPerp.getNorm();

        // same as distance to center of hub, even if rotated by 90 degrees
        return tangentialVelocity / fuelExitToHubPerp.getNorm();
    }

    private record FuelExitToHub(Transform3d transform, Rotation2d angle) {}

    public record ShootingParameters(double velocityRPM, double hoodAngleRad, double headingRad) {}
}
