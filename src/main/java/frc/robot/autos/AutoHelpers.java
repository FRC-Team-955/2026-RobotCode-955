package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AllianceFlipUtil;
import frc.robot.RobotState;
import frc.robot.shooting.ShootingKinematics;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

import java.util.function.Supplier;

import static frc.robot.subsystems.drive.DriveConstants.moveToConfig;

public class AutoHelpers {
    private static final double intermediateLinearTolerance = 0.1;
    private static final double intermediateAngularTolerance = Units.degreesToRadians(20);

    private static final RobotState robotState = RobotState.get();
    private static final ShootingKinematics shootingKinematics = ShootingKinematics.get();

    private static final Drive drive = Drive.get();

    public static Command intermediateWaypoint(Supplier<Pose2d> poseSupplier, DriveConstants.MoveToConstraints constraints) {
        return drive
                .moveTo(
                        () -> AllianceFlipUtil.apply(poseSupplier.get()),
                        constraints
                                .withFullSpeed(true)
                )
                .until(() -> robotState.isAtPoseWithTolerance(
                        poseSupplier.get(),
                        intermediateLinearTolerance,
                        intermediateAngularTolerance
                ));
    }

    public static Command intermediateWaypointWithAiming(Supplier<Translation2d> translationSupplier, DriveConstants.MoveToConstraints constraints) {
        return intermediateWaypoint(
                () -> new Pose2d(
                        AllianceFlipUtil.apply(translationSupplier.get()),
                        Rotation2d.fromRadians(shootingKinematics.getShootingParameters().headingRad())
                ),
                constraints.withApplyAimingFeedforward(true)
        );
    }

    public static Command finalWaypoint(Supplier<Pose2d> poseSupplier, DriveConstants.MoveToConstraints constraints) {
        return drive
                .moveTo(
                        () -> AllianceFlipUtil.apply(poseSupplier.get()),
                        constraints
                )
                .until(() -> robotState.isAtPoseWithTolerance(
                        poseSupplier.get(),
                        moveToConfig.linearPositionToleranceMeters().get(),
                        moveToConfig.angularPositionToleranceRad().get()
                ));
    }

    public static Command finalWaypointWithAiming(Supplier<Translation2d> translationSupplier, DriveConstants.MoveToConstraints constraints) {
        return finalWaypoint(
                () -> new Pose2d(
                        AllianceFlipUtil.apply(translationSupplier.get()),
                        Rotation2d.fromRadians(shootingKinematics.getShootingParameters().headingRad())
                ),
                constraints.withApplyAimingFeedforward(true)
        );
    }

    public static Command yDistanceInterpolatingWaypoint(
            Pose2d start,
            Pose2d end,
            double yDistanceToStartInterpolation,
            DriveConstants.MoveToConstraints constraints
    ) {
        return finalWaypoint(
                () -> {
                    double yDistance = Math.abs(new Transform2d(end, robotState.getPose()).getY());
                    yDistance -= 0.1; // Add a slight offset so that we actually reach the end position
                    // No clamping needed, Pose2d.interpolate will handle it
                    double initialInterp = 1.0 - (yDistance / yDistanceToStartInterpolation);
                    return start.interpolate(end, initialInterp);
                },
                constraints
        );
    }
}
