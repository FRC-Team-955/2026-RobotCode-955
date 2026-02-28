package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.AllianceFlipUtil;
import frc.lib.Bounds;
import frc.lib.network.LoggedTunableNumber;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.shooting.ShootingKinematics;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.gamepiecevision.GamePieceVision;
import frc.robot.subsystems.superintake.Superintake;

import java.util.function.Supplier;

import static frc.robot.subsystems.drive.DriveConstants.*;

public class AutoHelpers {
    private static final double intermediateLinearTolerance = 0.3;
    private static final double intermediateAngularTolerance = Units.degreesToRadians(45);

    private static final RobotState robotState = RobotState.get();
    private static final ShootingKinematics shootingKinematics = ShootingKinematics.get();
    private static final GamePieceVision gamePieceVision = GamePieceVision.get();

    private static final Drive drive = Drive.get();
    private static final Superintake superintake = Superintake.get();

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
            Translation2d start,
            Translation2d end,
            Rotation2d heading,
            double yDistanceToStartInterpolation
    ) {
        Rotation2d startToEndFacing = end.minus(start).getAngle();

        // note that robot pose needs to be flipped BACK to blue alliance
        // because the waypoint functions will flip it to red alliance if needed
        double yDistance = Math.abs(new Transform2d(new Pose2d(end, startToEndFacing), AllianceFlipUtil.apply(robotState.getPose())).getY());
        yDistance -= 0.1; // Add a slight offset so that we actually reach the end position
        // No clamping needed, Pose2d.interpolate will handle it
        double interp = 1.0 - (yDistance / yDistanceToStartInterpolation);
        return new Pose2d(start.interpolate(end, interp), heading);
    }

    public static Command yDistanceInterpolatingWaypoint(
            Translation2d start,
            Translation2d end,
            Rotation2d heading,
            double yDistanceToStartInterpolation,
            DriveConstants.MoveToConstraints constraints
    ) {
        return finalWaypoint(
                () -> yDistanceInterpolation(start, end, heading, yDistanceToStartInterpolation),
                constraints
        );
    }

    public static Command yDistanceInterpolatingWaypointWithAiming(
            Translation2d start,
            Translation2d end,
            Rotation2d heading,
            double yDistanceToStartInterpolation,
            DriveConstants.MoveToConstraints constraints
    ) {
        return finalWaypointWithAiming(
                () -> yDistanceInterpolation(start, end, heading, yDistanceToStartInterpolation).getTranslation(),
                constraints
        );
    }

    private static final DriveConstants.MoveToConstraints intakeConstraints = defaultMoveToConstraints
            .withMaxLinearVelocityMetersPerSec(new LoggedTunableNumber("AutoHelpers/Intake/MaxLinearVelocity", 2))
            .withMaxAngularAccelerationRadPerSecPerSec(new LoggedTunableNumber("AutoHelpers/Intake/MaxAngularAcceleration", 40.0));


    private static Command intakeFromNeutralZone(
            Bounds bounds,
            Supplier<Pose2d> poseSupplierIfNoGamePieces
    ) {
        return Commands.parallel(
                drive.moveTo(
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
                ),
                superintake.setGoal(Superintake.Goal.INTAKE).asProxy()
        );
    }

    private static final double neutralZoneXMin = FieldConstants.LinesVertical.neutralZoneNear + driveConfig.bumperLengthMeters() / 2.0;
    // Allow going over center line a little bit
    private static final double neutralZoneXMax = FieldConstants.LinesVertical.center;

    public static Command intakeFromLeftNeutralZone(Supplier<Pose2d> poseSupplierIfNoGamePieces) {
        return intakeFromNeutralZone(
                new Bounds(
                        neutralZoneXMin,
                        neutralZoneXMax,
                        FieldConstants.LinesHorizontal.center + driveConfig.bumperLengthMeters() / 2.0,
                        FieldConstants.fieldWidth - driveConfig.bumperLengthMeters() / 2.0
                ),
                poseSupplierIfNoGamePieces
        );
    }

    public static Command intakeFromRightNeutralZone(Supplier<Pose2d> poseSupplierIfNoGamePieces) {
        return intakeFromNeutralZone(
                new Bounds(
                        neutralZoneXMin,
                        neutralZoneXMax,
                        0.0 + driveConfig.bumperLengthMeters() / 2.0,
                        FieldConstants.LinesHorizontal.center - driveConfig.bumperLengthMeters() / 2.0
                ),
                poseSupplierIfNoGamePieces
        );
    }
}
