package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.commands.CommandsExt;
import frc.robot.FieldConstants;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;

import static frc.robot.autos.AutoHelpers.shootingConstraints;
import static frc.robot.subsystems.drive.DriveConstants.driveConfig;

public class AuraAuto extends Auto {
    private static final Superintake superintake = Superintake.get();
    private static final Superstructure superstructure = Superstructure.get();

    private static final Pose2d outpostPosition = new Pose2d(1.0, FieldConstants.Outpost.centerPoint.getY(), Rotation2d.k180deg);
    private static final Pose2d depotPosition = new Pose2d(1.0, FieldConstants.Depot.depotCenter.getY(), Rotation2d.k180deg);
    private static final Pose2d depotIntakePosition = new Pose2d(0.3, FieldConstants.Depot.depotCenter.getY(), Rotation2d.k180deg);

    public AuraAuto() {
        super(
                new Pose2d(FieldConstants.LinesVertical.starting - driveConfig.bumperLengthMeters(), FieldConstants.LinesHorizontal.center, Rotation2d.k180deg),
                build()
        );
    }

    private static Command build() {
        return CommandsExt.eagerSequence(
                // Move, intake, and shoot towards depot
                AutoHelpers.intakeFromDepotWhileShooting(shootingConstraints)
                        .withTimeout(5),

                // Move towards center
                AutoHelpers.yDistanceInterpolatingWaypoint(
                        new Translation2d(2.5, 5.5),
                        new Translation2d(2.5, FieldConstants.LinesHorizontal.center),
                        Rotation2d.kCW_90deg,
                        1.0,
                        shootingConstraints.withAiming(true)
                ),

                // Move towards outpost
                AutoHelpers.yDistanceInterpolatingWaypoint(
                        new Translation2d(2.5, 1.0),
                        new Translation2d(1.0, 1.0),
                        new Rotation2d(),
                        2.0,
                        shootingConstraints.withAiming(true)
                )

                //// Move forward to shooting position
                //AutoHelpers.finalWaypoint(() -> shootingPosition, defaultMoveToConstraints),
                //
                //// Shoot for 1.5 sec
                //Commands.parallel(
                //        superintake.setGoal(Superintake.Goal.INTAKE),
                //        superstructure.setGoal(Superstructure.Goal.SHOOT),
                //        AutoHelpers.finalWaypoint(() -> shootingPosition, defaultMoveToConstraints.withAiming(true))
                //).withTimeout(1.5),
                //
                //// Go to the outpost and wait at outpost for ~3 sec while intaking
                //Commands.parallel(
                //        superintake.setGoal(Superintake.Goal.INTAKE),
                //        superstructure.setGoal(Superstructure.Goal.SHOOT),
                //        AutoHelpers.finalWaypoint(() -> outpostPosition, defaultMoveToConstraints)
                //).withTimeout(4),
                //
                //// Move back to shooting position
                //Commands.parallel(
                //        superintake.setGoal(Superintake.Goal.INTAKE),
                //        superstructure.setGoal(Superstructure.Goal.SHOOT),
                //        AutoHelpers.finalWaypoint(() -> shootingPosition, defaultMoveToConstraints.withAiming(true))
                //).withTimeout(1.5),
                //
                //// Shoot for ~1.5 sec
                //Commands.parallel(
                //        superintake.setGoal(Superintake.Goal.INTAKE),
                //        superstructure.setGoal(Superstructure.Goal.SHOOT),
                //        AutoHelpers.finalWaypoint(() -> shootingPosition, defaultMoveToConstraints.withAiming(true))
                //).withTimeout(1.5),
                //
                //// Go to the depot while intaking
                //Commands.parallel(
                //        superintake.setGoal(Superintake.Goal.INTAKE),
                //        superstructure.setGoal(Superstructure.Goal.SHOOT),
                //        Commands.sequence(
                //                AutoHelpers.finalWaypoint(() -> depotPosition, AutoHelpers.intakeConstraints),
                //                AutoHelpers.finalWaypoint(() -> depotIntakePosition, AutoHelpers.intakeConstraints)
                //        )
                //).withTimeout(3),
                //
                //// Go back to shooting position
                //AutoHelpers.finalWaypoint(() -> shootingPosition, defaultMoveToConstraints.withAiming(true)),
                //
                //// Shoot
                //Commands.parallel(
                //        superintake.setGoal(Superintake.Goal.INTAKE),
                //        superstructure.setGoal(Superstructure.Goal.SHOOT),
                //        AutoHelpers.finalWaypoint(() -> shootingPosition, defaultMoveToConstraints.withAiming(true))
                //).withTimeout(5)
        );
    }
}
