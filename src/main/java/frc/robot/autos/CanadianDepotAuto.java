package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.CommandsExt;
import frc.robot.RobotState;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;

import static frc.robot.subsystems.drive.DriveConstants.defaultMoveToConstraints;
import static frc.robot.subsystems.drive.DriveConstants.shootingConstraints;

public class CanadianDepotAuto extends Auto {
    private static final Superintake superintake = Superintake.get();
    private static final Superstructure superstructure = Superstructure.get();
    private static final RobotState robotState = RobotState.get();

    private static final double startingY = ChoreoTraj.CanadianDepot_FirstPass.initialPoseBlue().getY();
    private static final Rotation2d startingRotation = ChoreoTraj.CanadianDepot_FirstPass.initialPoseBlue().getRotation();
    private static final Pose2d trenchEntrance = new Pose2d(3.5, startingY, Rotation2d.kCW_90deg);

    public CanadianDepotAuto() {
        super(
                new Pose2d(4.45, startingY, startingRotation),
                build()
        );
    }

    private static Command build() {
        return CommandsExt.eagerSequence(
                // move out of trench
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(
                        ChoreoTraj.CanadianDepot_FirstPass.initialPoseBlue().getX(),
                        startingY,
                        startingRotation
                ), defaultMoveToConstraints, false),

                // follow collection path
                Commands.parallel(
                        AutoHelpers.trajectory(ChoreoTraj.CanadianDepot_FirstPass),
                        superintake.setGoal(Superintake.Goal.INTAKE).until(() -> true)
                ),
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),

                // go over the bump
                AutoHelpers.goOverDepotSideBump(),

                // Shoot
                Commands.parallel(
                        superintake.intakeShootAlternate(),
                        superstructure.setGoal(Superstructure.Goal.SHOOT),
                        AutoHelpers.finalWaypoint(robotState::getPose, defaultMoveToConstraints, true)
                ).withTimeout(4.5),
                superstructure.setGoal(Superstructure.Goal.IDLE).until(() -> true),

                // Move to final trench entrance
                AutoHelpers.finalWaypoint(() -> trenchEntrance, defaultMoveToConstraints, false),

                // move out of trench
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(
                        ChoreoTraj.CanadianDepot_FirstPass.initialPoseBlue().getX(),
                        startingY,
                        startingRotation
                ), defaultMoveToConstraints, false),

                // follow collection path
                Commands.parallel(
                        AutoHelpers.trajectory(ChoreoTraj.CanadianDepot_SecondPath),
                        superintake.setGoal(Superintake.Goal.INTAKE).until(() -> true)
                ),

                // go over the bump
                AutoHelpers.goOverDepotSideBump(),

                Commands.race(superintake.setGoal(Superintake.Goal.INTAKE),
                        AutoHelpers.finalWaypoint(() -> new Pose2d(0.67, 5.965,
                                Rotation2d.k180deg), shootingConstraints, true)),
                Commands.parallel(
                        AutoHelpers.finalWaypoint(() -> new Pose2d(1.0, 5.5,
                                Rotation2d.k180deg), defaultMoveToConstraints, true),

                        superstructure.setGoal(Superstructure.Goal.SHOOT))

                //// Shoot
                //Commands.parallel(
                //        superintake.intakeShootAlternate(),
                //        superstructure.setGoal(Superstructure.Goal.SHOOT),
                //        AutoHelpers.finalWaypoint(robotState::getPose, defaultMoveToConstraints, true)
                //),
                //superstructure.setGoal(Superstructure.Goal.IDLE).until(() -> true)
        );
    }
}
