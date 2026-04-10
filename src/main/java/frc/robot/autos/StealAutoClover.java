package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.CommandsExt;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;

import static frc.robot.subsystems.drive.DriveConstants.defaultMoveToConstraints;
import static frc.robot.subsystems.drive.DriveConstants.shootingConstraints;

public class StealAutoClover extends Auto {
    private static final Pose2d starting = ChoreoTraj.Clover_Auto.initialPoseBlue();

    public StealAutoClover() {
        super(starting, build());
    }

    private static Command build() {
        return CommandsExt.eagerSequence(
                // plz actually preload the fuels
                Commands.parallel(
                        AutoHelpers.trajectory(ChoreoTraj.Clover_Auto$0).withAiming(),
                        superstructure.setGoal(Superstructure.Goal.SHOOT).withTimeout(3.0)),

                superstructure.setGoal(Superstructure.Goal.IDLE).until(() -> true),
                Commands.waitSeconds(2.0),


                AutoHelpers.trajectory(ChoreoTraj.Clover_Auto$1),
                AutoHelpers.trajectory(ChoreoTraj.Clover_Auto$2),
                AutoHelpers.checkWaypoint(ChoreoTraj.Clover_Auto$2::endPoseBlue, defaultMoveToConstraints, false)
                , Commands.race(superintake.setGoal(Superintake.Goal.INTAKE),
                        AutoHelpers.trajectory(ChoreoTraj.Clover_Auto$3)),
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),
                AutoHelpers.trajectory(ChoreoTraj.Clover_Auto$4),
                AutoHelpers.checkWaypoint(ChoreoTraj.Clover_Auto$4::endPoseBlue, defaultMoveToConstraints, false),
                superstructure.setGoal(Superstructure.Goal.SHOOT).until(() -> true),
                AutoHelpers.trajectory(ChoreoTraj.Clover_Auto$5).withAiming().withConstraints(shootingConstraints),
                superintake.setGoal(Superintake.Goal.INTAKE).until(() -> true),
                AutoHelpers.trajectory(ChoreoTraj.Clover_Auto$6)
                        .withAiming().withConstraints(shootingConstraints)
                , AutoHelpers.trajectory(ChoreoTraj.Clover_Auto$7)
                        .withAiming().withConstraints(shootingConstraints),
                //AutoHelpers.checkWaypoint(ChoreoTraj.Clover_Auto$6::endPoseBlue,
                //        shootingConstraints, true),
                Commands.waitSeconds(5.0)
                // superstructure.setGoal(Superstructure.Goal.SHOOT).until(() -> true)


        );
    }
}