package frc.robot.autos;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.CommandsExt;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;

import static frc.robot.autos.AutoHelpers.leftNeutralZoneBounds;
import static frc.robot.autos.AutoHelpers.rightNeutralZoneBounds;
import static frc.robot.subsystems.drive.DriveConstants.defaultMoveToConstraints;

public class AggressiveDoubleBumpAuto extends Auto {
    private static final Pose2d starting = new Pose2d(
            3.3,
            2.5,
            Rotation2d.fromDegrees(-135)
    );

    public AggressiveDoubleBumpAuto(boolean flipY) {
        super(flipY ? ChoreoAllianceFlipUtil.getMirrorY().flip(starting) : starting, build(flipY));
    }

    private static Command build(boolean flipY) {
        return CommandsExt.eagerSequence(

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

                // move out of trench
                superintake.setGoal(Superintake.Goal.INTAKE).until(() -> true),
                AutoHelpers.trajectory(ChoreoTraj.AggressiveOutpostBumpBump$1, flipY),

                // go to entrance to trench
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),
                AutoHelpers.checkWaypoint(
                        flipY
                                ? () -> ChoreoAllianceFlipUtil.getMirrorY().flip(ChoreoTraj.AggressiveOutpostBumpBump$1.endPoseBlue())
                                : ChoreoTraj.AggressiveOutpostBumpBump$1::endPoseBlue,
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
                                        : () -> starting,
                                defaultMoveToConstraints,
                                true),

                        // shoot

                        superstructure.setGoal(Superstructure.Goal.SHOOT).until(() -> true),
                        Commands.parallel(
                                superintake.intakeShootAlternate()
                        ).withTimeout(4.5).until(() -> !superstructure.isHasFuel())
                ),

                superstructure.setGoal(Superstructure.Goal.IDLE).until(() -> true),
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),


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

                // move out of trench
                AutoHelpers.trajectory(ChoreoTraj.AggressiveOutpostBumpBump$4, flipY),

                // follow intake path
                superintake.setGoal(Superintake.Goal.INTAKE).until(() -> true),
                AutoHelpers.trajectory(ChoreoTraj.AggressiveOutpostBumpBump$4, flipY),
                AutoHelpers.intakeOrTrajectory(ChoreoTraj.AggressiveOutpostBumpBump$5, flipY ? leftNeutralZoneBounds : rightNeutralZoneBounds, flipY),
                AutoHelpers.trajectory(ChoreoTraj.AggressiveOutpostBumpBump$6, flipY),

                // make sure at entrance of trench
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),
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
                        superstructure.setGoal(Superstructure.Goal.SHOOT).until(() -> true),
                        Commands.parallel(
                                superintake.intakeShootAlternate()
                        ).withTimeout(4.5).until(() -> !superstructure.isHasFuel())
                ),

                superstructure.setGoal(Superstructure.Goal.IDLE).until(() -> true),
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),

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
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),
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
                        superstructure.setGoal(Superstructure.Goal.SHOOT).until(() -> true),
                        Commands.parallel(
                                superintake.intakeShootAlternate()
                        ).withTimeout(4.5).until(() -> !superstructure.isHasFuel())
                )
        );
    }
}
