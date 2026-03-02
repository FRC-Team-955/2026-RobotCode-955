package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.AllianceFlipUtil;
import frc.lib.commands.CommandsExt;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;

import static frc.robot.subsystems.drive.DriveConstants.defaultMoveToConstraints;

public class CenterAuto {
    private static final RobotState robotState = RobotState.get();

    private static final Superintake superintake = Superintake.get();
    private static final Superstructure superstructure = Superstructure.get();

    private static final Pose2d startingPosition = new Pose2d(1.5, FieldConstants.LinesHorizontal.center, Rotation2d.kZero);
    private static final Pose2d shootingPosition = new Pose2d(2.5, FieldConstants.LinesHorizontal.center, Rotation2d.k180deg);
    private static final Pose2d outpostPosition = new Pose2d(1.0, FieldConstants.Outpost.centerPoint.getY(), Rotation2d.k180deg);
    private static final Pose2d depotPosition = new Pose2d(1.0, FieldConstants.Depot.depotCenter.getY(), Rotation2d.k180deg);
    private static final Pose2d depotIntakePosition = new Pose2d(0.3, FieldConstants.Depot.depotCenter.getY(), Rotation2d.k180deg);

    public static Command build() {
        return CommandsExt.eagerSequence(
                // Set starting pose
                robotState.setPose(() -> AllianceFlipUtil.apply(startingPosition)),

                // Move forward to shooting position
                AutoHelpers.finalWaypoint(() -> shootingPosition, defaultMoveToConstraints),

                // Shoot for 5 sec
                Commands.parallel(
                        superintake.setGoal(Superintake.Goal.INTAKE),
                        superstructure.setGoal(Superstructure.Goal.SHOOT),
                        AutoHelpers.finalWaypoint(() -> shootingPosition, defaultMoveToConstraints.withAiming(true))
                ).withTimeout(2),
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),
                superstructure.setGoal(Superstructure.Goal.IDLE).until(() -> true),

                // Go to the outpost
                AutoHelpers.finalWaypoint(() -> outpostPosition, defaultMoveToConstraints),

                // Wait at outpost for 3 sec
                Commands.waitSeconds(3),

                // Move back to shooting position
                AutoHelpers.finalWaypoint(() -> shootingPosition, defaultMoveToConstraints),

                // Shoot for 5 sec
                Commands.parallel(
                        superintake.setGoal(Superintake.Goal.INTAKE),
                        superstructure.setGoal(Superstructure.Goal.SHOOT),
                        AutoHelpers.finalWaypoint(() -> shootingPosition, defaultMoveToConstraints.withAiming(true))
                ).withTimeout(5),
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),
                superstructure.setGoal(Superstructure.Goal.IDLE).until(() -> true),

                // Go to the depot while intaking
                Commands.parallel(
                        superintake.setGoal(Superintake.Goal.INTAKE),
                        Commands.sequence(
                                AutoHelpers.finalWaypoint(() -> depotPosition, AutoHelpers.intakeConstraints),
                                AutoHelpers.finalWaypoint(() -> depotIntakePosition, AutoHelpers.intakeConstraints)
                        )
                ).withTimeout(3),
                //superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),

                //// Make sure we go back
                //AutoHelpers.intermediateWaypoint(() -> new Pose2d(
                //        FieldConstants.LinesVertical.center,
                //        FieldConstants.LinesHorizontal.center,
                //        Rotation2d.k180deg
                //), defaultMoveToConstraints),

                // Go back to shooting position
                AutoHelpers.finalWaypoint(() -> shootingPosition, defaultMoveToConstraints),

                // Shoot
                Commands.parallel(
                        superintake.setGoal(Superintake.Goal.INTAKE),
                        superstructure.setGoal(Superstructure.Goal.SHOOT),
                        AutoHelpers.finalWaypoint(() -> shootingPosition, defaultMoveToConstraints.withAiming(true))
                ).withTimeout(5)
        );
    }
}
