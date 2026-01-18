package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import frc.lib.Util;
import frc.lib.network.LoggedTunableNumber;
import frc.lib.subsystem.Periodic;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class ShootingKinematics implements Periodic {
    private static final LoggedTunableNumber robotVelocityScalar = new LoggedTunableNumber("ShootingKinematics/RobotVelocityScalar", 1.2);

    public static final Transform3d ballExitTransform = new Transform3d(
            new Translation3d(Units.inchesToMeters(-4.0), Units.inchesToMeters(-9.0), Units.inchesToMeters(15.0)),
            new Rotation3d()
    );
    private static final Translation3d hubTranslation = new Translation3d(4.6256194, 4.0346376, 1.8288);
    private static final InterpolatingDoubleTreeMap distanceToVelocity = new InterpolatingDoubleTreeMap();

    static {
        distanceToVelocity.put(1.5, 6.5);
        distanceToVelocity.put(2.0, 7.0);
        distanceToVelocity.put(4.0, 8.0);
        distanceToVelocity.put(5.0, 9.0);
    }

    private static final RobotState robotState = RobotState.get();

    @Getter
    private ShootingParameters shootingParameters = null;
    @Getter
    private boolean validShootingParameters = false;

    private final Alert noValidShootingParametersAlert = new Alert("Could not find valid shooting parameters.", Alert.AlertType.kInfo);

    private static ShootingKinematics instance;

    public static ShootingKinematics get() {
        if (instance == null)
            synchronized (ShootingKinematics.class) {
                instance = new ShootingKinematics();
            }

        return instance;
    }

    private ShootingKinematics() {
    }

    private boolean isValidHoodAngle(double hoodAngleRad) {
        return hoodAngleRad > Units.degreesToRadians(15) && hoodAngleRad < Units.degreesToRadians(90);
    }

    @Override
    public void periodicBeforeCommands() {
        validShootingParameters = updateShootingParameters();
        noValidShootingParametersAlert.set(!validShootingParameters);
    }

    private boolean updateShootingParameters() {
        Pose3d robotPose = new Pose3d(robotState.getPose());
        Pose3d ballExitPose = robotPose.transformBy(ballExitTransform);
        Logger.recordOutput("ShootingKinematics/BallExitPose", ballExitPose);

        Pose3d hubPose = new Pose3d(Util.flipIfNeeded(hubTranslation), new Rotation3d());
        Transform3d ballExitToHub = new Transform3d(ballExitPose, hubPose);

        double xyDist = ballExitToHub.getTranslation().toTranslation2d().getNorm();
        double zDist = ballExitToHub.getTranslation().getZ();
        double v0 = distanceToVelocity.get(xyDist);
        Logger.recordOutput("ShootingKinematics/XYDist", xyDist);
        Logger.recordOutput("ShootingKinematics/Stationary/Velocity", v0);

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
            Logger.recordOutput("ShootingKinematics/Stationary/Phi", phi_1);
            Logger.recordOutput("ShootingKinematics/Stationary/PhiAlternative", phi_2);
        } else if (isValidHoodAngle(phi_2)) {
            phi_stationary = phi_2;
            Logger.recordOutput("ShootingKinematics/Stationary/Phi", phi_2);
            Logger.recordOutput("ShootingKinematics/Stationary/PhiAlternative", phi_1);
        } else {
            return false;
        }

        double vx = v0 * Math.cos(phi_stationary);
        double vz = v0 * Math.sin(phi_stationary);

        // 2. Next, rotate shooting vector into field coordinates
        Rotation2d robotToHub = hubPose.getTranslation().toTranslation2d()
                .minus(robotPose.getTranslation().toTranslation2d())
                .getAngle();
        // We could use Translation2d to rotate it, but since vy = 0, it's simple enough
        // to just use trig
        double vy = vx * robotToHub.getSin();
        vx = vx * robotToHub.getCos();

        // 3. Now subtract robot velocity from stationary shooting velocity to get final
        // shooting vector
        ChassisSpeeds robotSpeeds = robotState.getMeasuredChassisSpeeds().times(robotVelocityScalar.get());
        vx -= robotSpeeds.vxMetersPerSecond;
        vy -= robotSpeeds.vyMetersPerSecond;

        Logger.recordOutput("ShootingKinematics/Velocity/X", vx);
        Logger.recordOutput("ShootingKinematics/Velocity/Y", vy);
        Logger.recordOutput("ShootingKinematics/Velocity/Z", vz);

        // 4. Now calculate phi, theta, and shooting magnitude from 3d shooting vector
        double v = Math.sqrt(vx * vx + vy * vy + vz * vz);
        double phi = Math.asin(vz / v);
        // TODO: account for robot to ball exit offset
        double theta = Math.atan2(vy, vx);
        Logger.recordOutput("ShootingKinematics/Velocity", v);
        Logger.recordOutput("ShootingKinematics/Phi", phi);
        Logger.recordOutput("ShootingKinematics/Theta", theta);

        shootingParameters = new ShootingParameters(v, phi, theta);
        return true;
    }

    public record ShootingParameters(double velocityMetersPerSec, double hoodAngleRad, double headingRad) {}
}
