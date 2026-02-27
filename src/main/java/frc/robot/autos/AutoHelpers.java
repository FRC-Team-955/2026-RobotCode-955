package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AllianceFlipUtil;
import frc.lib.Bounds;
import frc.lib.network.LoggedTunableNumber;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.shooting.ShootingKinematics;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.gamepiecevision.GamePieceVision;

import java.util.function.Supplier;

import static frc.robot.subsystems.drive.DriveConstants.*;

public class AutoHelpers {
    private static final double intermediateLinearTolerance = 0.1;
    private static final double intermediateAngularTolerance = Units.degreesToRadians(20);

    private static final RobotState robotState = RobotState.get();
    private static final ShootingKinematics shootingKinematics = ShootingKinematics.get();
    private static final GamePieceVision gamePieceVision = GamePieceVision.get();

    private static final Drive drive = Drive.get();

    public static Command intermediateWaypoint(Supplier<Pose2d> poseSupplier, DriveConstants.MoveToConstraints constraints) {
        return drive
                .moveTo(
                        () -> AllianceFlipUtil.apply(poseSupplier.get()),
                        constraints
                                .withFullSpeed(true)
                )
                .until(() -> robotState.isAtPoseWithTolerance(
                        AllianceFlipUtil.apply(poseSupplier.get()),
                        intermediateLinearTolerance,
                        intermediateAngularTolerance
                ));
    }

    public static Command intermediateWaypointWithAiming(Supplier<Translation2d> translationSupplier, DriveConstants.MoveToConstraints constraints) {
        Supplier<Pose2d> poseSupplier =
                () -> new Pose2d(
                        AllianceFlipUtil.apply(translationSupplier.get()),
                        Rotation2d.fromRadians(shootingKinematics.getShootingParameters().headingRad())
                );
        return drive
                .moveTo(
                        poseSupplier,
                        constraints
                                .withFullSpeed(true)
                                .withApplyAimingFeedforward(true)
                )
                .until(() -> robotState.isAtPoseWithTolerance(
                        poseSupplier.get(),
                        intermediateLinearTolerance,
                        intermediateAngularTolerance
                ));
    }

    public static Command finalWaypoint(Supplier<Pose2d> poseSupplier, DriveConstants.MoveToConstraints constraints) {
        return drive
                .moveTo(
                        () -> AllianceFlipUtil.apply(poseSupplier.get()),
                        constraints
                )
                .until(() -> robotState.isAtPoseWithTolerance(
                        AllianceFlipUtil.apply(poseSupplier.get()),
                        moveToConfig.linearPositionToleranceMeters().get(),
                        moveToConfig.angularPositionToleranceRad().get()
                ));
    }

    public static Command finalWaypointWithAiming(Supplier<Translation2d> translationSupplier, DriveConstants.MoveToConstraints constraints) {
        Supplier<Pose2d> poseSupplier =
                () -> new Pose2d(
                        AllianceFlipUtil.apply(translationSupplier.get()),
                        Rotation2d.fromRadians(shootingKinematics.getShootingParameters().headingRad())
                );
        return drive
                .moveTo(
                        poseSupplier,
                        constraints
                                .withApplyAimingFeedforward(true)
                )
                .until(() -> robotState.isAtPoseWithTolerance(
                        poseSupplier.get(),
                        moveToConfig.linearPositionToleranceMeters().get(),
                        moveToConfig.angularPositionToleranceRad().get()
                ));
    }

    public static Command finalWaypointWithAimingForever(Supplier<Translation2d> translationSupplier, DriveConstants.MoveToConstraints constraints) {
        return drive.moveTo(
                () -> new Pose2d(
                        AllianceFlipUtil.apply(translationSupplier.get()),
                        Rotation2d.fromRadians(shootingKinematics.getShootingParameters().headingRad())
                ),
                constraints
                        .withApplyAimingFeedforward(true)
        );
    }

    public static Pose2d yDistanceInterpolation(
            Pose2d start,
            Pose2d end,
            double yDistanceToStartInterpolation
    ) {
        // note that robot pose needs to be flipped BACK to blue alliance
        // because the waypoint functions will flip it to red alliance if needed
        double yDistance = Math.abs(new Transform2d(end, AllianceFlipUtil.apply(robotState.getPose())).getY());
        yDistance -= 0.1; // Add a slight offset so that we actually reach the end position
        // No clamping needed, Pose2d.interpolate will handle it
        double interp = 1.0 - (yDistance / yDistanceToStartInterpolation);
        return start.interpolate(end, interp);
    }

    public static Command yDistanceInterpolatingWaypoint(
            Pose2d start,
            Pose2d end,
            double yDistanceToStartInterpolation,
            DriveConstants.MoveToConstraints constraints
    ) {
        return finalWaypoint(
                () -> yDistanceInterpolation(start, end, yDistanceToStartInterpolation),
                constraints
        );
    }

    public static Command yDistanceInterpolatingWaypointWithAiming(
            Pose2d start,
            Pose2d end,
            double yDistanceToStartInterpolation,
            DriveConstants.MoveToConstraints constraints
    ) {
        return finalWaypointWithAiming(
                () -> yDistanceInterpolation(start, end, yDistanceToStartInterpolation).getTranslation(),
                constraints
        );
    }

    private static final DriveConstants.MoveToConstraints intakeConstraints = defaultMoveToConstraints
            .withMaxLinearVelocityMetersPerSec(new LoggedTunableNumber("AutoHelpers/Intake/MaxVelocity", 2.5));

    private static Command intakeFromNeutralZone(
            Bounds bounds,
            Supplier<Pose2d> poseSupplierIfNoGamePieces
    ) {
        return drive.moveTo(
                () -> {
                    for (var target : gamePieceVision.getBestTargets()) {
                        if (AllianceFlipUtil.apply(bounds).contains(target)) {
                            return new Pose2d(
                                    target,
                                    // Point towards target
                                    target.minus(robotState.getTranslation()).getAngle()
                            );
                        }
                    }
                    return AllianceFlipUtil.apply(poseSupplierIfNoGamePieces.get());
                },
                intakeConstraints
        );
    }

    public static Command intakeFromLeftNeutralZone(Supplier<Pose2d> poseSupplierIfNoGamePieces) {
        return intakeFromNeutralZone(
                new Bounds(
                        FieldConstants.LinesVertical.neutralZoneNear + driveConfig.bumperLengthMeters() / 2.0,
                        FieldConstants.LinesVertical.center - driveConfig.bumperLengthMeters() / 2.0,
                        FieldConstants.LinesHorizontal.center + driveConfig.bumperLengthMeters() / 2.0,
                        FieldConstants.fieldWidth - driveConfig.bumperLengthMeters() / 2.0
                ),
                poseSupplierIfNoGamePieces
        );
    }

    public static Command intakeFromRightNeutralZone(Supplier<Pose2d> poseSupplierIfNoGamePieces) {
        return intakeFromNeutralZone(
                new Bounds(
                        FieldConstants.LinesVertical.neutralZoneNear + driveConfig.bumperLengthMeters() / 2.0,
                        FieldConstants.LinesVertical.center - driveConfig.bumperLengthMeters() / 2.0,
                        0.0 + driveConfig.bumperLengthMeters() / 2.0,
                        FieldConstants.LinesHorizontal.center - driveConfig.bumperLengthMeters() / 2.0
                ),
                poseSupplierIfNoGamePieces
        );
    }
}
