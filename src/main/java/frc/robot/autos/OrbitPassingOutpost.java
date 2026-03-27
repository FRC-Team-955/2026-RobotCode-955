package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.commands.CommandsExt;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;

import static frc.robot.subsystems.drive.DriveConstants.defaultMoveToConstraints;

public class OrbitPassingOutpost extends Auto {
    private static final Superintake superintake = Superintake.get();
    private static final Superstructure superstructure = Superstructure.get();

    private static final Pose2d starting = ChoreoTraj.OrbitPassingOutpost_Collect.initialPoseBlue();

    public OrbitPassingOutpost() {
        super(
                new Pose2d(4.37, starting.getY(), starting.getRotation()),
                build()
        );
    }

    private static Command build() {
        return CommandsExt.eagerSequence(
                // move out of trench
                AutoHelpers.intermediateWaypoint(() -> starting, defaultMoveToConstraints, false),

                // move into position
                superintake.setGoal(Superintake.Goal.INTAKE).until(() -> true),
                AutoHelpers.trajectory(ChoreoTraj.OrbitPassingOutpost_Collect$0),

                // follow passing path
                superstructure.setGoal(Superstructure.Goal.SHOOT).until(() -> true),
                AutoHelpers.trajectory(ChoreoTraj.OrbitPassingOutpost_Collect$1).withAiming(),
                superstructure.setGoal(Superstructure.Goal.IDLE).until(() -> true),

                // follow collection path
                AutoHelpers.trajectory(ChoreoTraj.OrbitPassingOutpost_Collect$2),
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),

                // move to entrance to trench
                AutoHelpers.intermediateWaypoint(ChoreoTraj.OrbitPassingOutpost_Collect::endPoseBlue, defaultMoveToConstraints, false),

                // go through trench to shooting position
                AutoHelpers.intermediateWaypoint(ChoreoTraj.OrbitPassingOutpost_Score::initialPoseBlue, defaultMoveToConstraints, false),

                // move into position
                superintake.setGoal(Superintake.Goal.INTAKE).until(() -> true),
                AutoHelpers.trajectory(ChoreoTraj.OrbitPassingOutpost_Score$0),

                // +1000 aura
                superstructure.setGoal(Superstructure.Goal.SHOOT).until(() -> true),
                AutoHelpers.trajectory(ChoreoTraj.OrbitPassingOutpost_Score$1).withAiming(),
                AutoHelpers.aimWhileStationary()
        );
    }
}
