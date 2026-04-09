package frc.robot.autos;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.CommandsExt;
import frc.robot.subsystems.gamepiecevision.GamePieceVision;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;

import static frc.robot.subsystems.drive.DriveConstants.defaultMoveToConstraints;

public class OrbitNewAuto extends Auto {
    private static final GamePieceVision gamePieceVision = GamePieceVision.get();
    private static final Pose2d starting = ChoreoTraj.OrbitOutpostNew.initialPoseBlue();

    public OrbitNewAuto(boolean flipY) {
        super(flipY ? ChoreoAllianceFlipUtil.getMirrorY().flip(starting) : starting, build(flipY));
    }

    private static Command build(boolean flipY) {
        return CommandsExt.eagerSequence(
                // move out of trench
                AutoHelpers.trajectory(ChoreoTraj.OrbitOutpostNew$0, flipY),

                // follow intake path
                superintake.setGoal(Superintake.Goal.INTAKE).until(() -> true),
                AutoHelpers.trajectory(ChoreoTraj.OrbitOutpostNew$1, flipY),

                // go to entrance to trench
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),
                AutoHelpers.trajectory(ChoreoTraj.OrbitOutpostNew$2, flipY),
                AutoHelpers.checkWaypoint(
                        flipY
                                ? () -> ChoreoAllianceFlipUtil.getMirrorY().flip(ChoreoTraj.OrbitOutpostNew$2.endPoseBlue())
                                : ChoreoTraj.OrbitOutpostNew$2::endPoseBlue,
                        defaultMoveToConstraints,
                        false),

                // go through trench and shoot
                AutoHelpers.trajectory(ChoreoTraj.OrbitOutpostNew$3, flipY),
                superstructure.setGoal(Superstructure.Goal.SHOOT).until(() -> true),
                Commands.parallel(
                        superintake.intakeShootAlternate(),
                        drive.stop().withAiming()
                ).withTimeout(4.5),
                superstructure.setGoal(Superstructure.Goal.IDLE).until(() -> true),

                // move out of trench
                AutoHelpers.trajectory(ChoreoTraj.OrbitOutpostNew$4, flipY),

                // follow intake path
                Commands.either(
                        CommandsExt.eagerSequence(
                                superintake.setGoal(Superintake.Goal.INTAKE).until(() -> true),
                                AutoHelpers.trajectory(ChoreoTraj.OrbitOutpostNew$6, flipY)
                        ),
                        CommandsExt.eagerSequence(
                                superintake.setGoal(Superintake.Goal.INTAKE).until(() -> true),
                                AutoHelpers.trajectory(ChoreoTraj.OrbitOutpostNew$6, flipY)
                        ).until(
                                () -> gamePieceVision.getBestTarget().isPresent()
                        ).andThen(
                                AutoHelpers.intermediateWaypoint(
                                        () -> new Pose2d(
                                                gamePieceVision.getBestTarget().get().getX(),
                                                gamePieceVision.getBestTarget().get().getY(),
                                                gamePieceVision.getBestTarget().get().getAngle()
                                        ),
                                        defaultMoveToConstraints,
                                        false
                                )
                        ),
                        gamePieceVision::anyCamerasDisconnected
                ),

                // move to entrance of trench
                AutoHelpers.trajectory(ChoreoTraj.OrbitOutpostNew$6, flipY),

                // make sure at entrance of trench
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),
                AutoHelpers.checkWaypoint(
                        flipY
                                ? () -> ChoreoAllianceFlipUtil.getMirrorY().flip(ChoreoTraj.OrbitOutpostNew$6.endPoseBlue())
                                : ChoreoTraj.OrbitOutpostNew$7::endPoseBlue,
                        defaultMoveToConstraints,
                        false),

                // go through trench and shoot
                AutoHelpers.trajectory(ChoreoTraj.OrbitOutpostNew$7, flipY),
                superstructure.setGoal(Superstructure.Goal.SHOOT).until(() -> true),
                Commands.parallel(
                        superintake.intakeShootAlternate(),
                        drive.stop().withAiming()
                )
        );
    }
}
