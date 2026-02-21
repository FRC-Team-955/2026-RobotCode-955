package frc.robot;

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
import frc.robot.subsystems.superstructure.flywheel.Flywheel;
import frc.robot.subsystems.superstructure.hood.Hood;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class ShootingKinematics implements Periodic {
    private static final LoggedTunableNumber robotVelocityScalar = new LoggedTunableNumber("ShootingKinematics/RobotVelocityScalar", 1.2);
    private static final LoggedTunableNumber headingToleranceDeg = new LoggedTunableNumber("ShootingKinematics/HeadingToleranceDegrees", 5.0);
    public static final LoggedTunableNumber velocityToleranceRPM = new LoggedTunableNumber("ShootingKinematics/VelocityToleranceRPM", 100);
    public static final LoggedTunableNumber hoodToleranceDeg = new LoggedTunableNumber("ShootingKinematics/HoodToleranceDegrees", 1.0);

    public static final Translation3d fuelExitTranslation = new Translation3d(
            Units.inchesToMeters(-4.0),
            Units.inchesToMeters(-9.0),
            Units.inchesToMeters(15.0)
    );
    public static final Rotation2d fuelExitRotation = Rotation2d.k180deg;
    private static final InterpolatingDoubleTreeMap distanceToVelocity = new InterpolatingDoubleTreeMap();

    static {
        distanceToVelocity.put(1.5, 6.5);
        distanceToVelocity.put(2.0, 7.0);
        distanceToVelocity.put(4.0, 8.0);
        distanceToVelocity.put(5.0, 9.0);
    }

    private static final RobotState robotState = RobotState.get();
    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();

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
        Logger.recordOutput("ShootingKinematics/ShootingParameters", shootingParameters);
        Logger.recordOutput("ShootingKinematics/ValidShootingParameters", validShootingParameters);
        noValidShootingParametersAlert.set(!validShootingParameters);

        shootingParametersMet = validShootingParameters &&
                Math.abs(robotState.getPose().getRotation().getRadians() - shootingParameters.headingRad()) <= Units.degreesToRadians(headingToleranceDeg.get()) &&
                Math.abs(flywheel.getVelocityRPM() - shootingParameters.velocityRPM()) <= velocityToleranceRPM.get() &&
                Math.abs(hood.getPositionRad() - shootingParameters.hoodAngleRad()) <= Units.degreesToRadians(hoodToleranceDeg.get());
        Logger.recordOutput("ShootingKinematics/ShootingParametersMet", shootingParametersMet);
    }

    private static final LoggedTunableNumber shootHubManualFlywheelRPM = new LoggedTunableNumber("ShootingKinematics/ShootHubManual/FlywheelRPM", 5.0);
    private static final LoggedTunableNumber shootHubManualHoodDegrees = new LoggedTunableNumber("ShootingKinematics/ShootHubManual/HoodDegrees", 45.0);

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
        return hoodAngleRad > Units.degreesToRadians(15) && hoodAngleRad < Units.degreesToRadians(90);
    }

    private boolean updateShootingParametersAutomatic() {
        FuelExitToHub fuelExitToHub = getFuelExitToHub();

        double xyDist = fuelExitToHub.transform().getTranslation().toTranslation2d().getNorm();
        double zDist = fuelExitToHub.transform().getTranslation().getZ();
        double v0 = distanceToVelocity.get(xyDist);
//        Logger.recordOutput("ShootingKinematics/XYDist", xyDist);
//        Logger.recordOutput("ShootingKinematics/Stationary/Velocity", v0);

        // 1. Compute stationary shooting velocity
        // maple-sim uses 11 m/s² for gravity
        final double g = 10.4; //9.81;
        double discriminant = Math.pow(v0, 4) - g * (g * xyDist * xyDist + 2 * zDist * v0 * v0);
        if (discriminant < 0) {
            return false;
        }
        double phi_1 = Math.atan((v0 * v0 + Math.sqrt(discriminant)) / (g * xyDist));
        double phi_2 = Math.atan((v0 * v0 - Math.sqrt(discriminant)) / (g * xyDist));

        // Find largest and valid phi (largest guarantees we have a path that will fall
        // down into the hub instead of going straight at the hub)
        double phi_stationary;
        if (isValidHoodAngle(phi_1) && (phi_1 > phi_2 || !isValidHoodAngle(phi_2))) {
            phi_stationary = phi_1;
//            Logger.recordOutput("ShootingKinematics/Stationary/Phi", phi_1);
//            Logger.recordOutput("ShootingKinematics/Stationary/PhiAlternative", phi_2);
        } else if (isValidHoodAngle(phi_2)) {
            phi_stationary = phi_2;
//            Logger.recordOutput("ShootingKinematics/Stationary/Phi", phi_2);
//            Logger.recordOutput("ShootingKinematics/Stationary/PhiAlternative", phi_1);
        } else {
            return false;
        }

        double vx = v0 * Math.cos(phi_stationary);
        double vz = v0 * Math.sin(phi_stationary);

        // 2. Next, rotate shooting vector into field coordinates
        // Note that using fuel exit pose instead of robot pose automatically takes care
        // of compensating for theta difference when looking from center of robot and from
        // fuel exit point
        // We could use Translation2d to rotate it, but since vy = 0, it's simple enough
        // to just use trig
        double vy = vx * fuelExitToHub.angle().getSin();
        vx = vx * fuelExitToHub.angle().getCos();

        // 3. Now subtract robot velocity from stationary shooting velocity to get final
        // shooting vector
        ChassisSpeeds robotSpeeds = robotState.getMeasuredChassisSpeeds().times(robotVelocityScalar.get());
        Translation2d robotSpeedsRotated = new Translation2d(
                robotSpeeds.vxMetersPerSecond,
                robotSpeeds.vyMetersPerSecond
        ).rotateBy(fuelExitRotation);
        vx -= robotSpeedsRotated.getX();
        vy -= robotSpeedsRotated.getY();

        // 4. Account for drivebase angular velocity
        Vector<N3> fuelExitFieldRelative = new Translation3d(
                fuelExitTranslation.toTranslation2d()
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

        shootingParameters = new ShootingParameters(v, phi, theta);
        return true;
    }

    private FuelExitToHub getFuelExitToHub() {
        Pose2d robotPose2d = robotState.getPose();
        Pose3d fuelExitPose = new Pose3d(
                new Pose3d(robotPose2d)
                        .transformBy(new Transform3d(
                                fuelExitTranslation,
                                new Rotation3d()
                        ))
                        .getTranslation(),
                new Rotation3d(
                        robotPose2d.getRotation()
                                .plus(fuelExitRotation)
                )
        );

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

    private record FuelExitToHub(Transform3d transform, Rotation2d angle) {}

    public record ShootingParameters(double velocityRPM, double hoodAngleRad, double headingRad) {}
}
