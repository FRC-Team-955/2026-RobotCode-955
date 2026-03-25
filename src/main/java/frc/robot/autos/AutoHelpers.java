package frc.robot.autos;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.AllianceFlipUtil;
import frc.lib.Bounds;
import frc.lib.Util;
import frc.lib.commands.CommandsExt;
import frc.lib.network.LoggedTunableNumber;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.constraints.DriveConstraints;
import frc.robot.subsystems.gamepiecevision.GamePieceVision;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;

import java.util.Optional;
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
    private static final Choreo.TrajectoryCache trajectoryCache = new Choreo.TrajectoryCache();

    public static Command intermediateWaypoint(Supplier<Pose2d> poseSupplier, DriveConstraints constraints, boolean aiming) {
        var cmd = drive
                .moveTo(
                        () -> AllianceFlipUtil.apply(poseSupplier.get()),
                        constraints
                        //.withFullSpeed(true)
                );
        if (aiming) {
            cmd = cmd.withAiming();
        }
        return cmd.until(() -> robotState.isAtPoseWithTolerance(
                AllianceFlipUtil.apply(poseSupplier.get()),
                intermediateLinearTolerance,
                // if we are aiming, the rotation from the pose supplier
                // is not the rotation that move to will target
                aiming
                        ? Double.MAX_VALUE
                        : intermediateAngularTolerance
        ));
    }

    public static Command finalWaypoint(Supplier<Pose2d> poseSupplier, DriveConstraints constraints, boolean aiming) {
        var cmd = drive
                .moveTo(
                        () -> AllianceFlipUtil.apply(poseSupplier.get()),
                        constraints
                );
        if (aiming) {
            cmd = cmd.withAiming();
        }
        return cmd.until(() -> robotState.isAtPoseWithTolerance(
                AllianceFlipUtil.apply(poseSupplier.get()),
                finalLinearTolerance,
                // if we are aiming, the rotation from the pose supplier
                // is not the rotation that move to will target
                aiming
                        ? Double.MAX_VALUE
                        : finalAngularTolerance
        ));
    }

    @SuppressWarnings("unchecked")
    public static Drive.ModifiableDriveCommand trajectory(ChoreoTraj traj) {
        // Basically copied from AutoFactory
        Optional<? extends Trajectory<?>> optTrajectory;
        if (traj.segment().isPresent()) {
            optTrajectory = trajectoryCache.loadTrajectory(traj.name(), traj.segment().getAsInt());
        } else {
            optTrajectory = trajectoryCache.loadTrajectory(traj.name());
        }
        if (optTrajectory.isPresent()) {
            return drive.followTrajectory((Trajectory<SwerveSample>) optTrajectory.get());
        } else {
            Util.error("Trajectory " + traj.name() + " is not present");
            return drive.stop();
        }
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
            DriveConstraints constraints,
            boolean aiming
    ) {
        return intermediateWaypoint(
                () -> yDistanceInterpolation(start, end, heading, yDistanceToStartInterpolation),
                constraints,
                aiming
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
            DriveConstraints constraints,
            boolean aiming
    ) {
        return intermediateWaypoint(
                () -> xDistanceToStartInterpolation(start, end, heading, xDistanceToStartInterpolation),
                constraints,
                aiming
        );
    }

    public static final DriveConstraints intakeConstraints = defaultMoveToConstraints
            .withMaxLinearVelocityMetersPerSec(new LoggedTunableNumber("AutoHelpers/Intake/MaxLinearVelocity", 2))
            .withMaxAngularAccelerationRadPerSecPerSec(new LoggedTunableNumber("AutoHelpers/Intake/MaxAngularAcceleration", 40.0));

    private static final LinearFilter targetXFilter = LinearFilter.movingAverage(10);
    private static final LinearFilter targetYFilter = LinearFilter.movingAverage(10);

    public static Pose2d getIntakePose(Bounds bounds, Pose2d ifNoGamePieces) {
        for (var target : gamePieceVision.getBestTargetsInBounds(Optional.of(bounds))) {
            if (AllianceFlipUtil.apply(bounds).contains(target)) {
                Translation2d targetFiltered = new Translation2d(
                        targetXFilter.calculate(target.getX()),
                        targetYFilter.calculate(target.getY())
                );
                return new Pose2d(
                        targetFiltered,
                        // Point towards target
                        targetFiltered.minus(robotState.getTranslation()).getAngle()
                ).transformBy(new Transform2d(
                        // Move align pose towards robot so that it doesn't try to move
                        // such that the center of the robot is at the target
                        -(driveConfig.bumperLengthMeters() / 2.0),
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

    public static Command intakeFromDepotWhileShooting(DriveConstraints constraints) {
        Pose2d ifNoGamePieces = new Pose2d(0.2 + driveConfig.bumperLengthMeters() / 2.0, FieldConstants.Depot.depotCenter.getY(), Rotation2d.k180deg);
        return Commands.parallel(
                drive.moveTo(
                        () -> getIntakePose(new Bounds(
                                driveConfig.bumperLengthMeters() / 2.0,
                                FieldConstants.Depot.depth,
                                FieldConstants.Depot.rightCorner.getY(),
                                FieldConstants.Depot.leftCorner.getY()
                        ), ifNoGamePieces),
                        constraints
                ).withAiming(),
                superintake.setGoal(Superintake.Goal.INTAKE),
                superstructure.setGoal(Superstructure.Goal.SHOOT)
        );
    }

    private static final DriveConstraints bumpConstraints = defaultMoveToConstraints
            .withMaxLinearVelocityMetersPerSec(new LoggedTunableNumber("AutoHelpers/Bump/MaxLinearVelocity", 2.5));
    private static double bumpStartX = 5.71;
    private static double bumpEndX = 2.6;

    public static Command goOverDepotSideBump() {
        double y = 5.53;
        Rotation2d rotation = Rotation2d.fromDegrees(135);
        return CommandsExt.eagerSequence(
                // go to the start of the bump
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(
                        bumpStartX,
                        y,
                        rotation
                ), defaultMoveToConstraints, false),

                // go over the bump
                AutoHelpers.finalWaypoint(() -> new Pose2d(
                        bumpEndX,
                        y,
                        rotation
                ), bumpConstraints, false)
        );
    }
}
