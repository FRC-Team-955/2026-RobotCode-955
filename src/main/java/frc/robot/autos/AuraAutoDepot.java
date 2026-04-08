package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.CommandsExt;
import frc.robot.FieldConstants;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;

import static frc.robot.autos.AutoHelpers.intakeConstraints;
import static frc.robot.subsystems.drive.DriveConstants.driveConfig;
import static frc.robot.subsystems.drive.DriveConstants.shootingConstraints;

public class AuraAutoDepot extends Auto {
    private static final Pose2d outpostPosition = new Pose2d(1.0, FieldConstants.Outpost.centerPoint.getY(), Rotation2d.k180deg);
    private static final Pose2d depotPosition = new Pose2d(1.0, FieldConstants.Depot.depotCenter.getY(), Rotation2d.k180deg);
    private static final Pose2d depotIntakePosition = new Pose2d(0.3, FieldConstants.Depot.depotCenter.getY(), Rotation2d.k180deg);

    public AuraAutoDepot() {
        super(
                new Pose2d(FieldConstants.LinesVertical.starting - driveConfig.bumperLengthMeters() / 2.0, FieldConstants.LinesHorizontal.center, Rotation2d.k180deg),
                build()
        );
    }

    private static Command build() {
        return CommandsExt.eagerSequence(
                // Shoot while moving to depot


                Commands.race(superstructure.setGoal(Superstructure.Goal.SHOOT),
                        AutoHelpers.finalWaypoint(() -> new Pose2d(1.57, 5.0, Rotation2d.k180deg),
                                shootingConstraints, true), superintake.intakeShootAlternate()),
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),
                superstructure.setGoal(Superstructure.Goal.IDLE).until(() -> true),

                Commands.race(
                        superintake.setGoal(Superintake.Goal.INTAKE),
                        CommandsExt.eagerSequence(
                                AutoHelpers.finalWaypoint(()
                                        -> new Pose2d(0.48, 5.0, Rotation2d.kCCW_90deg), intakeConstraints, false),
                                AutoHelpers.finalWaypoint(
                                        () -> new Pose2d(0.48, 6.76, Rotation2d.kCCW_90deg),
                                        intakeConstraints,
                                        false
                                )

                        )
                ),
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),
                Commands.race(
                        AutoHelpers.yDistanceInterpolatingWaypoint(
                                new Translation2d(2.5, 7.14),
                                new Translation2d(2.5, 2.2),
                                Rotation2d.k180deg,
                                1.0,
                                shootingConstraints,
                                true
                        ),
                        superstructure.setGoal(Superstructure.Goal.SHOOT),
                        superintake.intakeShootAlternate()
                ),

                //Commands.race(
                //        superintake.intakeShootAlternate(),
                //        superstructure.setGoal(Superstructure.Goal.SHOOT),
                //        AutoHelpers.finalWaypoint(() -> new Pose2d(1.25, 7.2, Rotation2d.k180deg), shootingConstraints, true)
                //),
                //superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),
                //superstructure.setGoal(Superstructure.Goal.IDLE).until(() -> true),
                //
                //
                //
                //
                //superstructure.setGoal(Superstructure.Goal.IDLE).until(() -> true),
                //Commands.race(
                //        superintake.setGoal(Superintake.Goal.INTAKE),
                //        AutoHelpers.finalWaypoint(() -> new Pose2d(0.45, 5.9,
                //                Rotation2d.k180deg), intakeConstraints, false)
                //),

                //// Move toward depot
                //AutoHelpers.intermediateWaypoint(
                //        () -> new Pose2d(1.6, 5.0, Rotation2d.kCCW_90deg),
                //        intakeConstraints,
                //        false
                //),
                //
                //// Move to side of depot for easier intaking
                //AutoHelpers.finalWaypoint(
                //        () -> new Pose2d(0.5, 4.8, Rotation2d.kCCW_90deg),
                //        intakeConstraints,
                //        false
                //),
                //
                //// Intake the Depot
                //Commands.race(
                //        AutoHelpers.finalWaypoint(
                //                () -> new Pose2d(0.5, 6.5, Rotation2d.kCCW_90deg),
                //                intakeConstraints,
                //                false
                //        ),
                //        superintake.setGoal(Superintake.Goal.INTAKE)
                //),

                // Stop Intaking
                superstructure.setGoal(Superstructure.Goal.IDLE).until(() -> true),

                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),
                //AutoHelpers.finalWaypoint(() -> new Pose2d(
                //                2.0, 4.7, Rotation2d.kCW_90deg),
                //        defaultMoveToConstraints,
                //        false
                //),
                //// Shoot while moving to the outpost
                //Commands.race(
                //        AutoHelpers.yDistanceInterpolatingWaypoint(
                //                new Translation2d(2.5, 5.5),
                //                new Translation2d(2.5, FieldConstants.LinesHorizontal.center),
                //                Rotation2d.kCW_90deg,
                //                1.0,
                //                shootingConstraints,
                //                true
                //        ),
                //        superstructure.setGoal(Superstructure.Goal.SHOOT),
                //        superintake.intakeShootAlternate()
                //),
                //
                //// Moving backwards to outpost
                //AutoHelpers.xDistanceInterpolatingWaypoint(
                //        new Translation2d(2.5, 0.7),
                //        new Translation2d(1.0, 0.7),
                //        new Rotation2d(),
                //        1.5,
                //        shootingConstraints,
                //        true
                //
                //
                //),
                Commands.parallel(
                        superintake.setGoal(Superintake.Goal.INTAKE).until(() -> true),
                        AutoHelpers.trajectory(ChoreoTraj.AuraAutoDepotIntake)
                ),

                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),

                //// Line up to the Outpost
                //
                //AutoHelpers.yDistanceInterpolatingWaypoint(
                //
                //        new Translation2d(1.0, 0.7),
                //        new Translation2d(1.0, 0.55),
                //        Rotation2d.k180deg,
                //        1.0,
                //        intakeConstraints,
                //        false
                //),


                // Move back to idle position
                //  superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),

                // Move closer to the hub to shoot
                Commands.parallel(
                        AutoHelpers.finalWaypoint(
                                () -> new Pose2d(1.8, 1.5, Rotation2d.k180deg),
                                shootingConstraints,
                                true
                        ),
                        superintake.intakeShootAlternate(),
                        superstructure.setGoal(Superstructure.Goal.SHOOT)
                )
        );
    }
}
