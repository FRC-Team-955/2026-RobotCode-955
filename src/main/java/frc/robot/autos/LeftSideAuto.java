package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.AllianceFlipUtil;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;

public final class LeftSideAuto {
    private LeftSideAuto() {}

    private static final List<Pose2d> waypoints = List.of(
            new Pose2d(3.5, 5.5, Rotation2d.fromRadians(-Math.PI)),
            new Pose2d(4.586890697479248, 5.476860046386719, Rotation2d.fromRadians(-2.356194490192345)),
            new Pose2d(5.95119047164917, 5.457370281219482, Rotation2d.fromRadians(-1.5707963267948966)),
            new Pose2d(7.003650665283203, 5.437880039215088, Rotation2d.fromRadians(0.0)),
            new Pose2d(7.880700588226318, 5.418390274047852, Rotation2d.fromRadians(0.0)),
            new Pose2d(5.938254356384277, 7.395676612854004, Rotation2d.fromRadians(-1.5707963267948966)),
            new Pose2d(3.687418222427368, 7.34134578704834, Rotation2d.fromRadians(-1.5707963267948966)),
            new Pose2d(2.026456117630005, 6.495341777801514, Rotation2d.fromRadians(-2.356194490192345)),
            new Pose2d(0.9708915948867798, 6.821325302124023, Rotation2d.fromRadians(-1.5707963267948966)),
            new Pose2d(0.6526699066162109, 5.098270893096924, Rotation2d.fromRadians(-1.5707963267948966)),
            new Pose2d(2.2049708366394043, 4.7645263671875, Rotation2d.fromRadians(-Math.PI))
    );

    public static Command build(Drive drive, RobotState robotState) {
        List<Pose2d> poses = waypoints.stream()
                .map(AllianceFlipUtil::apply)
                .toList();

        Command setInitialPose = Commands.runOnce(() -> robotState.setPose(poses.get(0)), drive);

        AtomicInteger goalIndex = new AtomicInteger(1);
        AtomicReference<Pose2d> currentGoal = new AtomicReference<>(poses.get(Math.min(1, poses.size() - 1)));

        double switchLinearToleranceMeters = 0.5;
        double switchAngularToleranceRad = Math.toRadians(10);

        BooleanSupplier atFinalGoal = () -> {
            Pose2d goal = currentGoal.get();
            return goalIndex.get() >= poses.size()
                    && robotState.isAtPoseWithTolerance(
                    goal,
                    DriveConstants.moveToConfig.linearPositionToleranceMeters(),
                    DriveConstants.moveToConfig.angularPositionToleranceRad())
                    && robotState.isMeasuredChassisSpeedsBelowTolerance(
                    DriveConstants.moveToConfig.linearVelocityToleranceMetersPerSec(),
                    DriveConstants.moveToConfig.angularVelocityToleranceRadPerSec());
        };

        Command advanceGoals = Commands.run(() -> {
            Pose2d goal = currentGoal.get();
            boolean readyToAdvance = robotState.isAtPoseWithTolerance(
                    goal,
                    switchLinearToleranceMeters,
                    switchAngularToleranceRad);

            if (readyToAdvance && goalIndex.get() < poses.size()) {
                currentGoal.set(poses.get(goalIndex.getAndIncrement()));
            }
        }).until(atFinalGoal);
        Command followPath = drive.moveTo(currentGoal::get, false);
        return Commands.sequence(setInitialPose, Commands.deadline(advanceGoals, followPath));
    }
}
