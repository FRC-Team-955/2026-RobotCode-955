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
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.gamepiecevision.GamePieceVision;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;

import java.util.function.Supplier;

import static frc.robot.subsystems.drive.DriveConstants.defaultMoveToConstraints;
import static frc.robot.subsystems.drive.DriveConstants.driveConfig;

public class AutoHelpers {
    private static final double intermediateLinearTolerance = 1;
    private static final double intermediateAngularTolerance = Units.degreesToRadians(45);
    private static final double finalLinearTolerance = 0.1;
    private static final double finalAngularTolerance = Units.degreesToRadians(10);

    private static final RobotState robotState = RobotState.get();
    private static final GamePieceVision gamePieceVision = GamePieceVision.get();

    private static final Drive drive = Drive.get();
    private static final Superintake superintake = Superintake.get();
    private static final Superstructure superstructure = Superstructure.get();

    public static final DriveConstants.MoveToConstraints shootingConstraints = defaultMoveToConstraints
            .withMaxLinearVelocityMetersPerSec(new LoggedTunableNumber("AutoHelpers/Intake/MaxLinearVelocity", 1.0))
            .withAiming(true);

    public static Command intermediateWaypoint(Supplier<Pose2d> poseSupplier, DriveConstants.MoveToConstraints constraints) {
        return drive
                .moveTo(
                        () -> AllianceFlipUtil.apply(poseSupplier.get()),
                        constraints
                        //.withFullSpeed(true)
                )
                .until(() -> robotState.isAtPoseWithTolerance(
                        AllianceFlipUtil.apply(poseSupplier.get()),
                        intermediateLinearTolerance,
                        // if we are aiming, the rotation from the pose supplier
                        // is not the rotation that move to will target
                        constraints.aiming()
                                ? Double.MAX_VALUE
                                : intermediateAngularTolerance
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
                        finalLinearTolerance,
                        // if we are aiming, the rotation from the pose supplier
                        // is not the rotation that move to will target
                        constraints.aiming()
                                ? Double.MAX_VALUE
                                : intermediateAngularTolerance
                ));
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

    public static Pose2d xDistanceToStartInterpolation(
            Translation2d start,
            Translation2d end,
            Rotation2d heading,
            double xDistanceToStartInterpolation
    ) {
        Rotation2d startToEndFacing = end.minus(start).getAngle();

        // note that robot pose needs to be flipped BACK to blue alliance
        // because the waypoint functions will flip it to red alliance if needed
        double xDistance = Math.abs(new Transform2d(new Pose2d(end, startToEndFacing), AllianceFlipUtil.apply(robotState.getPose())).getX());
        xDistance -= 0.1; // Add a slight offset so that we actually reach the end position
        // No clamping needed, Pose2d.interpolate will handle it
        double interp = 1.0 - (xDistance / xDistanceToStartInterpolation);
        return new Pose2d(start.interpolate(end, interp), heading);
    }


    public static Command xDistanceInterpolatingWaypoint(
            Translation2d start,
            Translation2d end,
            Rotation2d heading,
            double xDistanceToStartInterpolation,
            DriveConstants.MoveToConstraints constraints
    ) {
        return finalWaypoint(
                () -> xDistanceToStartInterpolation(start, end, heading, xDistanceToStartInterpolation),
                constraints
        );
    }

    public static final DriveConstants.MoveToConstraints intakeConstraints = defaultMoveToConstraints
            .withMaxLinearVelocityMetersPerSec(new LoggedTunableNumber("AutoHelpers/Intake/MaxLinearVelocity", 2))
            .withMaxAngularAccelerationRadPerSecPerSec(new LoggedTunableNumber("AutoHelpers/Intake/MaxAngularAcceleration", 40.0));

    private static Pose2d getIntakePose(Bounds bounds, Pose2d ifNoGamePieces) {
        for (var target : gamePieceVision.getBestTargets()) {
            if (AllianceFlipUtil.apply(bounds).contains(target)) {
                return new Pose2d(
                        target,
                        // Point towards target
                        target.minus(robotState.getTranslation()).getAngle()
                ).transformBy(new Transform2d(
                        // Move align pose towards robot so that it doesn't try to move
                        // such that the center of the robot is at the target
                        -(0.1 + driveConfig.bumperLengthMeters() / 2.0),
                        0.0,
                        new Rotation2d()
                ));
            }
        }
        return AllianceFlipUtil.apply(ifNoGamePieces);
    }

    private static final double neutralZoneXMin = FieldConstants.LinesVertical.neutralZoneNear + driveConfig.bumperLengthMeters() / 2.0;
    private static final double neutralZoneXMax = FieldConstants.LinesVertical.center - 0.2;

    public static Command intakeFromLeftNeutralZone(Supplier<Pose2d> poseSupplierIfNoGamePieces) {
        return Commands.parallel(
                drive.moveTo(
                        () -> getIntakePose(new Bounds(
                                neutralZoneXMin,
                                neutralZoneXMax,
                                FieldConstants.LinesHorizontal.center,
                                FieldConstants.fieldWidth - driveConfig.bumperLengthMeters() / 2.0
                        ), poseSupplierIfNoGamePieces.get()),
                        intakeConstraints
                ),
                superintake.setGoal(Superintake.Goal.INTAKE)
        );
    }

    public static Command intakeFromRightNeutralZone(Supplier<Pose2d> poseSupplierIfNoGamePieces) {
        return Commands.parallel(
                drive.moveTo(
                        () -> getIntakePose(new Bounds(
                                neutralZoneXMin,
                                neutralZoneXMax,
                                0.0 + driveConfig.bumperLengthMeters() / 2.0,
                                FieldConstants.LinesHorizontal.center
                        ), poseSupplierIfNoGamePieces.get()),
                        intakeConstraints
                ),
                superintake.setGoal(Superintake.Goal.INTAKE)
        );
    }

    public static Command intakeFromDepotWhileShooting(DriveConstants.MoveToConstraints constraints) {
        Pose2d ifNoGamePieces = new Pose2d(0.2 + driveConfig.bumperLengthMeters() / 2.0, FieldConstants.Depot.depotCenter.getY(), Rotation2d.k180deg);
        return Commands.parallel(
                drive.moveTo(
                        () -> getIntakePose(new Bounds(
                                driveConfig.bumperLengthMeters() / 2.0,
                                FieldConstants.Depot.depth,
                                FieldConstants.Depot.rightCorner.getY(),
                                FieldConstants.Depot.leftCorner.getY()
                        ), ifNoGamePieces),
                        constraints.withAiming(true)
                ),
                superintake.setGoal(Superintake.Goal.INTAKE),
                superstructure.setGoal(Superstructure.Goal.SHOOT)
        );
    }
}
