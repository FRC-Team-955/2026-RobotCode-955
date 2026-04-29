package frc.robot.autos;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.CommandsExt;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;

import static frc.robot.autos.AutoHelpers.leftNeutralZoneBounds;
import static frc.robot.autos.AutoHelpers.rightNeutralZoneBounds;
import static frc.robot.subsystems.drive.DriveConstants.defaultMoveToConstraints;

public class PassiveAuto extends Auto {
    private static final Pose2d starting = ChoreoTraj.PassiveOutpost.initialPoseBlue();

    public PassiveAuto(boolean flipY) {
        super(flipY ? ChoreoAllianceFlipUtil.getMirrorY().flip(starting) : starting, build(flipY));
    }

    private static Command build(boolean flipY) {
        return CommandsExt.eagerSequence(
                // move out of trench
                superintake.setGoal(Superintake.Goal.INTAKE).until(() -> true),
                AutoHelpers.trajectory(ChoreoTraj.PassiveOutpost$0, flipY),

                // follow intake path
                AutoHelpers.trajectory(ChoreoTraj.PassiveOutpost$1, flipY),

                // go to entrance to trench
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),
                AutoHelpers.trajectory(ChoreoTraj.PassiveOutpost$2, flipY),
                AutoHelpers.checkWaypoint(
                        flipY
                                ? () -> ChoreoAllianceFlipUtil.getMirrorY().flip(ChoreoTraj.PassiveOutpost$2.endPoseBlue())
                                : ChoreoTraj.PassiveOutpost$2::endPoseBlue,
                        defaultMoveToConstraints,
                        false),

                // go through trench and shoot
                AutoHelpers.trajectory(ChoreoTraj.PassiveOutpost$3, flipY),
                superstructure.setGoal(Superstructure.Goal.SHOOT).until(() -> true),
                Commands.parallel(
                        superintake.intakeShootAlternate(),
                        drive.stop().withAiming()
                ).withTimeout(4.5).until(() -> !superstructure.isHasFuel()),
                superstructure.setGoal(Superstructure.Goal.IDLE).until(() -> true),

                // move out of trench
                AutoHelpers.trajectory(ChoreoTraj.PassiveOutpost$4, flipY),

                // follow intake path
                superintake.setGoal(Superintake.Goal.INTAKE).until(() -> true),
                AutoHelpers.trajectory(ChoreoTraj.PassiveOutpost$5, flipY),
                AutoHelpers.intakeOrTrajectory(ChoreoTraj.PassiveOutpost$6, flipY ? leftNeutralZoneBounds : rightNeutralZoneBounds, flipY),
                AutoHelpers.trajectory(ChoreoTraj.PassiveOutpost$7, flipY),

                // make sure at entrance of trench
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),
                AutoHelpers.checkWaypoint(
                        flipY
                                ? () -> ChoreoAllianceFlipUtil.getMirrorY().flip(ChoreoTraj.PassiveOutpost$7.endPoseBlue())
                                : ChoreoTraj.PassiveOutpost$7::endPoseBlue,
                        defaultMoveToConstraints,
                        false
                ),

                // go through trench and shoot
                AutoHelpers.trajectory(ChoreoTraj.PassiveOutpost$8, flipY),
                superstructure.setGoal(Superstructure.Goal.SHOOT).until(() -> true),
                Commands.parallel(
                        superintake.intakeShootAlternate(),
                        drive.stop().withAiming()
                )
        );
    }
}
