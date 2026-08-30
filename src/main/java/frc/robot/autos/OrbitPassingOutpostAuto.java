package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.commands.CommandsExt;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;

import static frc.robot.subsystems.drive.DriveConstants.defaultMoveToConstraints;

public class OrbitPassingOutpostAuto extends Auto {
    private static final Pose2d starting = ChoreoTraj.OrbitPassingOutpost$1.initialPoseBlue();

    public OrbitPassingOutpostAuto() {
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
                AutoHelpers.trajectory(ChoreoTraj.OrbitPassingOutpost$1),

                // follow passing path
                superstructure.setGoal(Superstructure.Goal.SHOOT).until(() -> true),
                AutoHelpers.trajectory(ChoreoTraj.OrbitPassingOutpost$2).withAiming(),
                superstructure.setGoal(Superstructure.Goal.IDLE).until(() -> true),

                // follow collection path
                AutoHelpers.trajectory(ChoreoTraj.OrbitPassingOutpost$3),
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),

                // move to entrance to trench
                AutoHelpers.intermediateWaypoint(ChoreoTraj.OrbitPassingOutpost$3::endPoseBlue, defaultMoveToConstraints, false),

                // go through trench to shooting position
                AutoHelpers.intermediateWaypoint(ChoreoTraj.OrbitPassingOutpost$5::initialPoseBlue, defaultMoveToConstraints, false),

                // move into position
                superintake.setGoal(Superintake.Goal.INTAKE).until(() -> true),
                AutoHelpers.trajectory(ChoreoTraj.OrbitPassingOutpost$5),

                // +1000 aura
                superstructure.setGoal(Superstructure.Goal.SHOOT).until(() -> true),
                AutoHelpers.trajectory(ChoreoTraj.OrbitPassingOutpost$6).withAiming(),
                drive.stop().withAiming()
        );
    }
}
