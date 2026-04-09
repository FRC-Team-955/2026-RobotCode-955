package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.CommandsExt;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;

import static frc.robot.subsystems.drive.DriveConstants.defaultMoveToConstraints;

public class OrbitOutpostNewAuto extends Auto {
    private static final Pose2d starting = ChoreoTraj.OrbitOutpostNew.initialPoseBlue();

    public OrbitOutpostNewAuto() {
        super(starting, build());
    }

    private static Command build() {
        return CommandsExt.eagerSequence(
                // move out of trench
                AutoHelpers.trajectory(ChoreoTraj.OrbitOutpostNew$0),

                // follow intake path
                superintake.setGoal(Superintake.Goal.INTAKE).until(() -> true),
                AutoHelpers.trajectory(ChoreoTraj.OrbitOutpostNew$1),

                // go to entrance to trench
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),
                AutoHelpers.trajectory(ChoreoTraj.OrbitOutpostNew$2),
                AutoHelpers.checkWaypoint(ChoreoTraj.OrbitOutpostNew$2::endPoseBlue, defaultMoveToConstraints, false),

                // go through trench and shoot
                AutoHelpers.trajectory(ChoreoTraj.OrbitOutpostNew$3),
                superstructure.setGoal(Superstructure.Goal.SHOOT).until(() -> true),
                Commands.parallel(
                        superintake.intakeShootAlternate(),
                        drive.stop().withAiming()
                ).withTimeout(4.5),
                superstructure.setGoal(Superstructure.Goal.IDLE).until(() -> true),

                // move out of trench
                AutoHelpers.trajectory(ChoreoTraj.OrbitOutpostNew$4),

                // follow intake path
                superintake.setGoal(Superintake.Goal.INTAKE).until(() -> true),
                AutoHelpers.trajectory(ChoreoTraj.OrbitOutpostNew$5),

                // go to entrance to trench
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),
                AutoHelpers.checkWaypoint(ChoreoTraj.OrbitOutpostNew$5::endPoseBlue, defaultMoveToConstraints, false),

                // go through trench and shoot
                AutoHelpers.trajectory(ChoreoTraj.OrbitOutpostNew$6),
                superstructure.setGoal(Superstructure.Goal.SHOOT).until(() -> true),
                Commands.parallel(
                        superintake.intakeShootAlternate(),
                        drive.stop().withAiming()
                )
        );
    }
}
