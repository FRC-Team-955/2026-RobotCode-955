package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.CommandsExt;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;

import static frc.robot.autos.AutoHelpers.intakeConstraints;
import static frc.robot.subsystems.drive.DriveConstants.defaultMoveToConstraints;
import static frc.robot.subsystems.drive.DriveConstants.driveConfig;

public class AuraAuto extends Auto {
    private static final Drive drive = Drive.get();
    private static final Superintake superintake = Superintake.get();
    private static final Superstructure superstructure = Superstructure.get();

    private static final Pose2d outpostPosition = new Pose2d(1.0, FieldConstants.Outpost.centerPoint.getY(), Rotation2d.k180deg);
    private static final Pose2d depotPosition = new Pose2d(1.0, FieldConstants.Depot.depotCenter.getY(), Rotation2d.k180deg);
    private static final Pose2d depotIntakePosition = new Pose2d(0.3, FieldConstants.Depot.depotCenter.getY(), Rotation2d.k180deg);

    public AuraAuto() {
        super(
                new Pose2d(FieldConstants.LinesVertical.starting - driveConfig.bumperLengthMeters() / 2.0, FieldConstants.LinesHorizontal.center, Rotation2d.k180deg),
                build()
        );
    }

    private static Command build() {
        return CommandsExt.eagerSequence(
                Commands.deadline(
                        AutoHelpers.finalWaypoint(
                                () -> new Pose2d(2.0, 5.0, Rotation2d.kZero),
                                defaultMoveToConstraints,
                                true
                        ),
                        superstructure.setGoal(Superstructure.Goal.SHOOT)
                ),
                superstructure.setGoal(Superstructure.Goal.IDLE).until(() -> true),

                AutoHelpers.intermediateWaypoint(
                        () -> new Pose2d(1.6, 5.0, Rotation2d.kCCW_90deg),
                        intakeConstraints,
                        false
                ),

                AutoHelpers.finalWaypoint(
                        () -> new Pose2d(0.5, 4.8, Rotation2d.kCCW_90deg),
                        intakeConstraints,
                        false
                ),

                Commands.race(
                        AutoHelpers.finalWaypoint(
                                () -> new Pose2d(0.5, 6.5, Rotation2d.kCCW_90deg),
                                intakeConstraints,
                                false
                        ),
                        superintake.setGoal(Superintake.Goal.INTAKE)
                ),
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),

                // Move, intake, and shoot towards depot
                //AutoHelpers.intakeFromDepotWhileShooting(shootingConstraints)
                //        .withTimeout(5),
                //
                //// Move towards center
                superstructure.setGoal(Superstructure.Goal.SHOOT).until(() -> true),
                AutoHelpers.yDistanceInterpolatingWaypoint(
                        new Translation2d(2.5, 5.5),
                        new Translation2d(2.5, FieldConstants.LinesHorizontal.center),
                        Rotation2d.kCW_90deg,
                        1.0,
                        defaultMoveToConstraints,
                        true
                ),

                Commands.race(
                        AutoHelpers.yDistanceInterpolatingWaypoint(
                                new Translation2d(2.5, 1.0),
                                new Translation2d(1.0, 1.0),
                                new Rotation2d(),
                                2.0,
                                defaultMoveToConstraints,
                                true
                        ),
                        superintake.intakeShootAlternate()
                ),

                superintake.setGoal(Superintake.Goal.INTAKE).until(() -> true),
                superstructure.setGoal(Superstructure.Goal.IDLE).until(() -> true),
                AutoHelpers.finalWaypoint(
                        () -> new Pose2d(1.1, 0.6, Rotation2d.k180deg),
                        defaultMoveToConstraints,
                        false
                ),
                Commands.waitSeconds(4.0),
                Commands.parallel(
                        //AutoHelpers.finalWaypoint(
                        //        () -> new Pose2d(1.5, 1.45, Rotation2d.kCCW_90deg),
                        //        defaultMoveToConstraints,
                        //        true
                        //),
                        AutoHelpers.aimWhileStationary(),
                        superintake.intakeShootAlternate(),
                        superstructure.setGoal(Superstructure.Goal.SHOOT)
                )

                //drive.moveTo(
                //        () -> AutoHelpers.getIntakePose(
                //                new Bounds(
                //                        driveConfig.bumperLengthMeters() / 2.0,
                //                        1.4,
                //                        driveConfig.bumperWidthMeters() / 2.0,
                //                        1.8
                //                ),
                //                AutoHelpers.yDistanceInterpolation(
                //                        new Translation2d(2.5, 1.0),
                //                        new Translation2d(1.0, 1.0),
                //                        new Rotation2d(),
                //                        2.0
                //                )
                //        ),
                //        shootingConstraints
                //)

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