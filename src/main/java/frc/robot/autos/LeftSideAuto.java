package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.AllianceFlipUtil;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.ShootingKinematics;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;

import static frc.robot.subsystems.drive.DriveConstants.defaultMoveToConstraints;

public final class LeftSideAuto {
    private LeftSideAuto() {
    }

    private static final List<Pose2d> baseWaypoints = List.of(
            new Pose2d(3.5, 5.5, Rotation2d.fromDegrees(-90)),
            new Pose2d(4.2, 5.5, Rotation2d.fromDegrees(-45)),
            new Pose2d(4.6, 5.5, Rotation2d.fromDegrees(0)),
            new Pose2d(5.0, 5.5, Rotation2d.fromDegrees(45)),
            new Pose2d(6.0, 5.5, Rotation2d.fromDegrees(0)),
            new Pose2d(7.0, 5.5, Rotation2d.fromDegrees(0)),
            new Pose2d(8.0, 5.5, Rotation2d.fromDegrees(0)),
            new Pose2d(6.0, 7.5, Rotation2d.fromDegrees(-90)),
            new Pose2d(3.5, 7.5, Rotation2d.fromDegrees(-90)),
            new Pose2d(2.5, 7.5, Rotation2d.fromDegrees(180))
    );

    private static final int INTERPOLATION_START_INDEX = baseWaypoints.size();

    private static final Drive drive = Drive.get();
    private static final RobotState robotState = RobotState.get();
    private static final Superintake superintake = Superintake.get();
    private static final Superstructure superstructure = Superstructure.get();

    public static Pose2d getStartingPose() {
        return AllianceFlipUtil.apply(baseWaypoints.get(0));
    }

    public static Command build() {
        List<Pose2d> poses = baseWaypoints.stream()
                .map(AllianceFlipUtil::apply)
                .toList();

        Translation2d hubCenter = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
        Translation2d depotCenter = AllianceFlipUtil.apply(FieldConstants.Depot.depotCenter.toTranslation2d());
        Pose2d lastBaseWaypoint = poses.get(poses.size() - 1);
        Translation2d interpolationStartPos = lastBaseWaypoint.getTranslation();

        Translation2d depotApproachPos = AllianceFlipUtil.apply(new Translation2d(
                FieldConstants.Depot.depotCenter.getX() + 0.5,
                FieldConstants.Depot.depotCenter.getY()
        ));

        Command setInitialPose = Commands.runOnce(() -> robotState.setPose(poses.get(0)));

        AtomicInteger goalIndex = new AtomicInteger(1);
        AtomicReference<Pose2d> currentGoal = new AtomicReference<>(poses.get(Math.min(1, poses.size() - 1)));

        AtomicReference<Double> interpolationT = new AtomicReference<>(0.0);

        double totalInterpolationDistance = interpolationStartPos.getDistance(depotApproachPos);

        double switchLinearToleranceMeters = 0.5;
        double switchAngularToleranceRad = Math.toRadians(10);

        BooleanSupplier atFinalGoal = () -> {
            boolean interpolationComplete = interpolationT.get() >= 1.0;
            if (!interpolationComplete) return false;

            return robotState.isAtPoseWithTolerance(
                    currentGoal.get(),
                    DriveConstants.moveToConfig.linearPositionToleranceMeters().get(),
                    Math.PI * 2)
                    && robotState.isMeasuredChassisSpeedsBelowTolerance(
                    DriveConstants.moveToConfig.linearVelocityToleranceMetersPerSec().get(),
                    DriveConstants.moveToConfig.angularVelocityToleranceRadPerSec().get());
        };

        AtomicReference<Boolean> onInterpolationLine = new AtomicReference<>(false);

        Runnable advanceGoalsRunnable = () -> {
            boolean isOnInterpolationLine = goalIndex.get() >= INTERPOLATION_START_INDEX;
            onInterpolationLine.set(isOnInterpolationLine);

            if (isOnInterpolationLine) {
                Translation2d currentPos = robotState.getPose().getTranslation();
                double distanceFromStart = currentPos.getDistance(interpolationStartPos);
                double t = Math.min(distanceFromStart / totalInterpolationDistance, 1.0);
                interpolationT.set(t);

                Translation2d targetOnHubDepotLine = hubCenter.interpolate(depotCenter, t);
                Logger.recordOutput("LeftSideAuto/TargetOnHubDepotLine", targetOnHubDepotLine);
                Translation2d startToDepot = depotApproachPos.minus(interpolationStartPos);
                Translation2d perpendicular = new Translation2d(-startToDepot.getY(), startToDepot.getX())
                        .div(startToDepot.getNorm());

                double curveHeight = 2.5;
                Translation2d midpoint = interpolationStartPos.interpolate(depotApproachPos, 0.5);
                Translation2d controlPoint = midpoint.plus(perpendicular.times(curveHeight));
                Logger.recordOutput("LeftSideAuto/ControlPoint", controlPoint);

                double oneMinusT = 1.0 - t;
                Translation2d interpPrediction = interpolationStartPos.times(oneMinusT * oneMinusT)
                        .plus(controlPoint.times(2 * oneMinusT * t))
                        .plus(depotApproachPos.times(t * t));

                Logger.recordOutput("LeftSideAuto/InterpPrediction", interpPrediction);
                currentGoal.set(new Pose2d(interpPrediction, Rotation2d.fromDegrees(180)));

                Logger.recordOutput("LeftSideAuto/InterpolationT", t);
                Logger.recordOutput("LeftSideAuto/DistanceFromStart", distanceFromStart);
            } else {
                Pose2d goal = currentGoal.get();
                boolean readyToAdvance = robotState.isAtPoseWithTolerance(
                        goal,
                        switchLinearToleranceMeters,
                        switchAngularToleranceRad);

                if (readyToAdvance && goalIndex.get() < poses.size()) {
                    currentGoal.set(poses.get(goalIndex.getAndIncrement()));
                }

                if (readyToAdvance && goalIndex.get() == poses.size()) {
                    goalIndex.incrementAndGet();
                }
            }

            Logger.recordOutput("LeftSideAuto/GoalIndex", goalIndex.get());
            Logger.recordOutput("LeftSideAuto/OnInterpolationLine", isOnInterpolationLine);
            Logger.recordOutput("LeftSideAuto/CurrentGoal", currentGoal.get());
        };

        Runnable intakeWhileMovingRunnable = () -> {
            int index = goalIndex.get();
            if ((index >= 1 && index <= 8) || index >= INTERPOLATION_START_INDEX) {
                superintake.setGoal(Superintake.Goal.INTAKE).initialize();
            } else {
                superintake.setGoal(Superintake.Goal.IDLE).initialize();
            }
        };

        Runnable aimWhileOnInterpolationLineRunnable = () -> {
            if (onInterpolationLine.get()) {
                superstructure.setGoal(Superstructure.Goal.SHOOT).initialize();
                Logger.recordOutput("LeftSideAuto/Aiming", true);
            } else {
                Logger.recordOutput("LeftSideAuto/Aiming", false);
            }
        };

        Command shootAfterFinalGoal = Commands.deadline(
                Commands.sequence(
                        Commands.waitUntil(() -> ShootingKinematics.get().isShootingParametersMet()),
                        Commands.waitSeconds(2.0)
                ),
                drive.driveJoystickWithAiming(),
                superstructure.setGoal(Superstructure.Goal.SHOOT)
        ).onlyWhile(DriverStation::isAutonomousEnabled);

        Command driveMoveTo = drive.moveTo(currentGoal::get, defaultMoveToConstraints);
        Command driveMoveToWithAiming = drive.moveTo(() -> {
            Pose2d goal = currentGoal.get();
            return new Pose2d(goal.getTranslation(), Rotation2d.fromRadians(ShootingKinematics.get().getShootingParameters().headingRad()));
        }, defaultMoveToConstraints);

        return Commands.sequence(
                setInitialPose,
                Commands.deadline(
                        Commands.run(advanceGoalsRunnable).until(onInterpolationLine::get),
                        driveMoveTo,
                        Commands.run(intakeWhileMovingRunnable, superintake),
                        Commands.run(aimWhileOnInterpolationLineRunnable, superstructure)
                ),
                Commands.deadline(
                        Commands.run(advanceGoalsRunnable).until(atFinalGoal),
                        driveMoveToWithAiming,
                        Commands.run(intakeWhileMovingRunnable, superintake),
                        Commands.run(aimWhileOnInterpolationLineRunnable, superstructure)
                ),
                shootAfterFinalGoal
        );
    }
}
