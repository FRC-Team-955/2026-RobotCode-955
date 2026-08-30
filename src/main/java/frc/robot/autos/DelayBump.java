package frc.robot.autos;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.CommandsExt;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;

import static frc.robot.autos.AutoHelpers.leftNeutralZoneBounds;
import static frc.robot.autos.AutoHelpers.rightNeutralZoneBounds;
import static frc.robot.subsystems.drive.DriveConstants.defaultMoveToConstraints;

public class DelayBump extends Auto {
    private static final Pose2d starting = new Pose2d(
            3.3,
            2.5,
            Rotation2d.fromDegrees(-45)
    );

    public DelayBump(boolean flipY) {
        super(flipY ? ChoreoAllianceFlipUtil.getMirrorY().flip(new Pose2d(3.3, 2.5, Rotation2d.fromDegrees(-45))) : starting, build(flipY));
    }

    private static Command build(boolean flipY) {
        return CommandsExt.eagerSequence(
                Commands.parallel(
                        Superintake.getInstance().intakeShootAlternate(),
                        Superstructure.getInstance().setGoal(Superstructure.Goal.SHOOT),
                        Drive.getInstance().stop().withAiming()
                ).withTimeout(1.2),
                Superintake.getInstance().setGoal(Superintake.Goal.IDLE).until(() -> true),
                Superstructure.getInstance().setGoal(Superstructure.Goal.IDLE).until(() -> true),
                // go over the bump
                flipY
                        ? AutoHelpers.goOverDepotSideBump(true)
                        : AutoHelpers.goOverOutpostSideBump(true),

                AutoHelpers.checkWaypoint(
                        flipY
                                ? () -> ChoreoAllianceFlipUtil.getMirrorY().flip(
                                new Pose2d(
                                        5.71,
                                        2.5,
                                        Rotation2d.fromDegrees(-45)
                                )
                        )
                                : () -> ChoreoAllianceFlipUtil.getMirrorY().flip(
                                new Pose2d(
                                        5.71,
                                        5.5,
                                        Rotation2d.fromDegrees(-45)
                                )
                        ), defaultMoveToConstraints,
                        false
                ),

                // move out of trench
                Superintake.getInstance().setGoal(Superintake.Goal.INTAKE).until(() -> true),
                AutoHelpers.trajectory(ChoreoTraj.Delaybump$1, flipY),

                // go to entrance to trench
                Superintake.getInstance().setGoal(Superintake.Goal.IDLE).until(() -> true),
                AutoHelpers.checkWaypoint(
                        flipY
                                ? () -> ChoreoAllianceFlipUtil.getMirrorY().flip(ChoreoTraj.Delaybump$1.endPoseBlue())
                                : ChoreoTraj.Delaybump$1::endPoseBlue,
                        defaultMoveToConstraints,
                        false),

                // go over the bump
                flipY
                        ? AutoHelpers.goOverDepotSideBump(false)
                        : AutoHelpers.goOverOutpostSideBump(false),

                Commands.parallel(
                        AutoHelpers.checkWaypoint(
                                flipY
                                        ? () -> ChoreoAllianceFlipUtil.getMirrorY().flip(starting)
                                        : () -> new Pose2d(2, 3, Rotation2d.fromDegrees(20)),
                                defaultMoveToConstraints,
                                true),

                        // shoot

                        Superstructure.getInstance().setGoal(Superstructure.Goal.SHOOT).until(() -> true),
                        Commands.parallel(
                                Superintake.getInstance().intakeShootAlternate()
                        ).withTimeout(4.5).until(() -> !Superstructure.getInstance().isHasFuel())
                ),

                Superstructure.getInstance().setGoal(Superstructure.Goal.IDLE).until(() -> true),
                Superintake.getInstance().setGoal(Superintake.Goal.IDLE).until(() -> true),


                // go over the bump
                flipY
                        ? AutoHelpers.goOverDepotSideBump(true)
                        : AutoHelpers.goOverOutpostSideBump(true),

                AutoHelpers.checkWaypoint(
                        flipY
                                ? () -> ChoreoAllianceFlipUtil.getMirrorY().flip(
                                new Pose2d(
                                        5.71,
                                        2.5,
                                        Rotation2d.fromDegrees(-45)
                                )
                        )
                                : () -> ChoreoAllianceFlipUtil.getMirrorY().flip(
                                new Pose2d(
                                        5.71,
                                        5.5,
                                        Rotation2d.fromDegrees(-45)
                                )
                        ), defaultMoveToConstraints,
                        false
                ),

                // move out of trench
                AutoHelpers.trajectory(ChoreoTraj.Delaybump$2, flipY),

                // follow intake path
                Superintake.getInstance().setGoal(Superintake.Goal.INTAKE).until(() -> true),
                AutoHelpers.trajectory(ChoreoTraj.AggressiveOutpostBumpBump$4, flipY),
                AutoHelpers.intakeOrTrajectory(ChoreoTraj.AggressiveOutpostBumpBump$5, flipY ? leftNeutralZoneBounds : rightNeutralZoneBounds, flipY),
                AutoHelpers.trajectory(ChoreoTraj.AggressiveOutpostBumpBump$6, flipY),

                // make sure at entrance of trench
                Superintake.getInstance().setGoal(Superintake.Goal.IDLE).until(() -> true),
                AutoHelpers.checkWaypoint(
                        flipY
                                ? () -> ChoreoAllianceFlipUtil.getMirrorY().flip(ChoreoTraj.AggressiveOutpostBumpBump$6.endPoseBlue())
                                : ChoreoTraj.AggressiveOutpostBumpBump$6::endPoseBlue,
                        defaultMoveToConstraints,
                        false
                ),

                // go over the bump
                flipY
                        ? AutoHelpers.goOverDepotSideBump(false)
                        : AutoHelpers.goOverOutpostSideBump(false),

                Commands.parallel(
                        AutoHelpers.checkWaypoint(
                                flipY
                                        ? () -> ChoreoAllianceFlipUtil.getMirrorY().flip(starting)
                                        : () -> starting,
                                defaultMoveToConstraints,
                                true),

                        // shoot
                        Superstructure.getInstance().setGoal(Superstructure.Goal.SHOOT).until(() -> true),
                        Commands.parallel(
                                Superintake.getInstance().intakeShootAlternate()
                        ).withTimeout(4.5).until(() -> !Superstructure.getInstance().isHasFuel())
                ),

                Superstructure.getInstance().setGoal(Superstructure.Goal.IDLE).until(() -> true),
                Superintake.getInstance().setGoal(Superintake.Goal.IDLE).until(() -> true),

                // go over the bump
                flipY
                        ? AutoHelpers.goOverDepotSideBump(true)
                        : AutoHelpers.goOverOutpostSideBump(true),

                AutoHelpers.checkWaypoint(
                        flipY
                                ? () -> ChoreoAllianceFlipUtil.getMirrorY().flip(
                                new Pose2d(
                                        5.71,
                                        2.5,
                                        Rotation2d.fromDegrees(-135)
                                )
                        )
                                : () -> ChoreoAllianceFlipUtil.getMirrorY().flip(
                                new Pose2d(
                                        5.71,
                                        5.5,
                                        Rotation2d.fromDegrees(-135)
                                )
                        ), defaultMoveToConstraints,
                        false
                ),

                AutoHelpers.trajectory(ChoreoTraj.AggressiveOutpostBumpBump$9, flipY),

                // make sure at entrance of trench
                Superintake.getInstance().setGoal(Superintake.Goal.IDLE).until(() -> true),
                AutoHelpers.checkWaypoint(
                        flipY
                                ? () -> ChoreoAllianceFlipUtil.getMirrorY().flip(ChoreoTraj.AggressiveOutpostBumpBump$9.endPoseBlue())
                                : ChoreoTraj.AggressiveOutpostBumpBump$9::endPoseBlue,
                        defaultMoveToConstraints,
                        false
                ),

                // go over the bump
                flipY
                        ? AutoHelpers.goOverDepotSideBump(false)
                        : AutoHelpers.goOverOutpostSideBump(false),

                Commands.parallel(
                        AutoHelpers.checkWaypoint(
                                flipY
                                        ? () -> ChoreoAllianceFlipUtil.getMirrorY().flip(starting)
                                        : () -> starting,
                                defaultMoveToConstraints,
                                true),

                        // shoot
                        Superstructure.getInstance().setGoal(Superstructure.Goal.SHOOT).until(() -> true),
                        Commands.parallel(
                                Superintake.getInstance().intakeShootAlternate()
                        ).withTimeout(4.5).until(() -> !Superstructure.getInstance().isHasFuel())
                )
        );
    }
}
