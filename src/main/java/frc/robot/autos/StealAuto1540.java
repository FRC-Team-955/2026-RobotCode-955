package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.CommandsExt;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;

import static frc.robot.subsystems.drive.DriveConstants.defaultMoveToConstraints;

public class StealAuto1540 extends Auto {
    private static final Superintake superintake = Superintake.get();
    private static final Superstructure superstructure = Superstructure.get();

    private static final double startingY = ChoreoTraj.CanadianOutpost_FirstPass.initialPoseBlue().getY();
    private static final Rotation2d startingRotation = ChoreoTraj.CanadianOutpost_FirstPass.initialPoseBlue().getRotation();
    private static final Pose2d preemptiveTrenchEntrance = new Pose2d(3.2, 0.6, Rotation2d.kCCW_90deg);
    private static final Pose2d trenchEntrance = new Pose2d(3.5, startingY, Rotation2d.kCCW_90deg);
    private static final Pose2d trenchEntranceNochange = new Pose2d(3.5, startingY, new Rotation2d());


    public StealAuto1540() {
        super(
                new Pose2d(4.45, startingY, startingRotation),
                build()
        );
    }

    private static Command build() {
        return CommandsExt.eagerSequence(
                // move out of trench
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(
                        ChoreoTraj.CanadianOutpost_FirstPass.initialPoseBlue().getX(),
                        startingY,
                        startingRotation
                ), defaultMoveToConstraints, false),

                // follow collection path
                Commands.parallel(
                        AutoHelpers.trajectory(ChoreoTraj.CanadianOutpost_FirstPass),
                        superintake.setGoal(Superintake.Goal.INTAKE).until(() -> true)
                ),

                Commands.parallel(
                        AutoHelpers.goOverOutpostSideBump()
                ),

                //To stop shoot on move
                AutoHelpers.finalWaypoint(() -> preemptiveTrenchEntrance, defaultMoveToConstraints, true),

                // Shoot while moving to entrance to trench
                Commands.parallel(
                        superintake.intakeShootAlternate(),
                        superstructure.setGoal(Superstructure.Goal.SHOOT),
                        AutoHelpers.finalWaypoint(() -> preemptiveTrenchEntrance, defaultMoveToConstraints, true)
                ).withTimeout(4.5),
                superstructure.setGoal(Superstructure.Goal.IDLE).until(() -> true),

                // Move to final trench entrance
                AutoHelpers.finalWaypoint(() -> trenchEntrance, defaultMoveToConstraints, false),

                //Move out for second path
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(
                        ChoreoTraj.CanadianOutpost_SecondPass.initialPoseBlue().getX(),
                        startingY,
                        startingRotation
                ), defaultMoveToConstraints, false),

                //Follow the second path
                Commands.parallel(
                        AutoHelpers.trajectory(ChoreoTraj.CanadianOutpost_SecondPass),
                        superintake.setGoal(Superintake.Goal.INTAKE).until(() -> true)
                ),

                Commands.parallel(
                        AutoHelpers.goOverOutpostSideBump()
                ),

                //To stop shoot on move
                AutoHelpers.finalWaypoint(() -> preemptiveTrenchEntrance, defaultMoveToConstraints, true),


                // Shoot while moving to entrance to trench
                Commands.parallel(
                        superintake.intakeShootAlternate(),
                        superstructure.setGoal(Superstructure.Goal.SHOOT),
                        AutoHelpers.finalWaypoint(() -> preemptiveTrenchEntrance, defaultMoveToConstraints, true)
                ).withTimeout(4.5)


        );
    }
}
