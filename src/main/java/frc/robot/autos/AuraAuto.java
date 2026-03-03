package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.CommandsExt;
import frc.robot.FieldConstants;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;

import static frc.robot.subsystems.drive.DriveConstants.defaultMoveToConstraints;

public class AuraAuto extends Auto {
    private static final Superintake superintake = Superintake.get();
    private static final Superstructure superstructure = Superstructure.get();

    private static final Pose2d shootingPosition = new Pose2d(2.5, FieldConstants.LinesHorizontal.center, Rotation2d.k180deg);
    private static final Pose2d outpostPosition = new Pose2d(1.0, FieldConstants.Outpost.centerPoint.getY(), Rotation2d.k180deg);
    private static final Pose2d depotPosition = new Pose2d(1.0, FieldConstants.Depot.depotCenter.getY(), Rotation2d.k180deg);
    private static final Pose2d depotIntakePosition = new Pose2d(0.3, FieldConstants.Depot.depotCenter.getY(), Rotation2d.k180deg);

    public AuraAuto() {
        super(
                new Pose2d(1.5, FieldConstants.LinesHorizontal.center, Rotation2d.kZero),
                build()
        );
    }

    private static Command build() {
        return CommandsExt.eagerSequence(
                // Move forward to shooting position
                AutoHelpers.finalWaypoint(() -> shootingPosition, defaultMoveToConstraints),

                // Shoot for 1.5 sec
                Commands.parallel(
                        superintake.setGoal(Superintake.Goal.INTAKE),
                        superstructure.setGoal(Superstructure.Goal.SHOOT),
                        AutoHelpers.finalWaypoint(() -> shootingPosition, defaultMoveToConstraints.withAiming(true))
                ).withTimeout(1.5),

                // Go to the outpost and wait at outpost for ~3 sec while intaking
                Commands.parallel(
                        superintake.setGoal(Superintake.Goal.INTAKE),
                        superstructure.setGoal(Superstructure.Goal.SHOOT),
                        AutoHelpers.finalWaypoint(() -> outpostPosition, defaultMoveToConstraints)
                ).withTimeout(4),

                // Move back to shooting position
                Commands.parallel(
                        superintake.setGoal(Superintake.Goal.INTAKE),
                        superstructure.setGoal(Superstructure.Goal.SHOOT),
                        AutoHelpers.finalWaypoint(() -> shootingPosition, defaultMoveToConstraints.withAiming(true))
                ).withTimeout(1.5),

                // Shoot for ~1.5 sec
                Commands.parallel(
                        superintake.setGoal(Superintake.Goal.INTAKE),
                        superstructure.setGoal(Superstructure.Goal.SHOOT),
                        AutoHelpers.finalWaypoint(() -> shootingPosition, defaultMoveToConstraints.withAiming(true))
                ).withTimeout(1.5),

                // Go to the depot while intaking
                Commands.parallel(
                        superintake.setGoal(Superintake.Goal.INTAKE),
                        superstructure.setGoal(Superstructure.Goal.SHOOT),
                        Commands.sequence(
                                AutoHelpers.finalWaypoint(() -> depotPosition, AutoHelpers.intakeConstraints),
                                AutoHelpers.finalWaypoint(() -> depotIntakePosition, AutoHelpers.intakeConstraints)
                        )
                ).withTimeout(3),

                // Go back to shooting position
                AutoHelpers.finalWaypoint(() -> shootingPosition, defaultMoveToConstraints.withAiming(true)),

                // Shoot
                Commands.parallel(
                        superintake.setGoal(Superintake.Goal.INTAKE),
                        superstructure.setGoal(Superstructure.Goal.SHOOT),
                        AutoHelpers.finalWaypoint(() -> shootingPosition, defaultMoveToConstraints.withAiming(true))
                ).withTimeout(5)
        );
    }
}
