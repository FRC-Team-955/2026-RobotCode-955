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

import static frc.robot.subsystems.drive.DriveConstants.defaultMoveToConstraints;

public class OrbitAtDepotSecondAuto extends Auto {
    private static final double startingPositionY = 7.45;
    private static final Pose2d trenchShootingPosition = new Pose2d(3.6, 7.45, Rotation2d.kCCW_90deg);
    public static final double exitTrenchX = 6.5;
    public static final double firstPassYOffset = 0.25;
    public static final double secondPassYOffset = 1.0;


    public OrbitAtDepotSecondAuto() {
        super(
                new Pose2d(4.35, startingPositionY, Rotation2d.kCW_90deg),
                build()
        );
    }

    private static Command build() {
        return CommandsExt.eagerSequence(
                // move out of trench
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(
                        exitTrenchX,
                        startingPositionY,
                        Rotation2d.kCW_90deg
                ), defaultMoveToConstraints, false),

                // intake, go to neturalzone
                superintake.setGoal(Superintake.Goal.INTAKE).until(() -> true),
                AutoHelpers.yDistanceInterpolatingWaypoint(
                        new Translation2d(FieldConstants.LinesVertical.center, 7.0),
                        new Translation2d(FieldConstants.LinesVertical.center, 6.4),
                        Rotation2d.kCW_90deg,
                        2,
                        defaultMoveToConstraints,
                        false
                ).withTimeout(3),

                //move to netruazone middle
                AutoHelpers.finalWaypoint(() -> new Pose2d(
                        FieldConstants.LinesVertical.center,
                        4.7,
                        Rotation2d.kCW_90deg
                ), AutoHelpers.intakeConstraints, false).withTimeout(3),
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),

                AutoHelpers.intermediateWaypoint(() -> new Pose2d(FieldConstants.LinesVertical.center -
                                1.0, 4.7,
                                Rotation2d.kCCW_90deg),
                        defaultMoveToConstraints,
                        false

                ),
                //   avoid scattering balls
                //AutoHelpers.finalWaypoint(() -> new Pose2d(
                //        FieldConstants.LinesVertical.center - 1.5,
                //        trenchShootingPosition.getY() - 0.15,
                //        trenchShootingPosition.getRotation()
                //), defaultMoveToConstraints, false).withTimeout(3),
                ////AutoHelpers.yDistanceInterpolatingWaypoint(
                //        new Translation2d(6.0, 5.0),
                //        new Translation2d(6.0, startingPositionY),
                //        Rotation2d.kCCW_90deg,
                //        1.75,
                //        defaultMoveToConstraints,
                //        false
                //),
                //),


                //move to entrance to trench
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(
                        6.1,
                        startingPositionY,
                        trenchShootingPosition.getRotation()
                ), defaultMoveToConstraints, false),

                // go through trench to shooting position
                AutoHelpers.finalWaypoint(() -> trenchShootingPosition, defaultMoveToConstraints, false),

                // shoot
                Commands.parallel(
                        superintake.intakeShootAlternate(),
                        superstructure.setGoal(Superstructure.Goal.SHOOT),
                        AutoHelpers.finalWaypoint(() -> trenchShootingPosition, defaultMoveToConstraints, true)
                ).withTimeout(4.5),
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),
                superstructure.setGoal(Superstructure.Goal.IDLE).until(() -> true),

                // move out of trench
                AutoHelpers.finalWaypoint(() -> new Pose2d(
                        exitTrenchX,
                        trenchShootingPosition.getY(),
                        trenchShootingPosition.getRotation()
                ), defaultMoveToConstraints, false),

                // intake
                Commands.parallel(
                        superintake.setGoal(Superintake.Goal.INTAKE).until(() -> true),
                        AutoHelpers.trajectory(ChoreoTraj.OrbitDepot)
                ),
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),
                AutoHelpers.finalWaypoint(() -> new Pose2d(
                        FieldConstants.LinesVertical.center - 1.5,
                        trenchShootingPosition.getY() - 0.15,
                        trenchShootingPosition.getRotation()
                ), defaultMoveToConstraints, false).withTimeout(3),

                //AutoHelpers.yDistanceInterpolatingWaypoint(
                //        new Translation2d(FieldConstants.LinesVertical.center - secondPassYOffset, 7.0),
                //        new Translation2d(FieldConstants.LinesVertical.center - secondPassYOffset, 4.8),
                //        Rotation2d.kCW_90deg,
                //        2,
                //        defaultMoveToConstraints,
                //        false
                //),
                //
                //
                ////move to netruazone middle
                //AutoHelpers.finalWaypoint(() -> new Pose2d(
                //        FieldConstants.LinesVertical.center - secondPassYOffset,
                //        3.7,
                //        Rotation2d.kCW_90deg
                //), AutoHelpers.intakeConstraints, false),


                //AutoHelpers.yDistanceInterpolatingWaypoint(
                //        new Translation2d(6.0, 4.5),
                //        //new Translation2d(6.0, startingPositionY),
                //        new Translation2d(6.0, trenchShootingPosition.getY() + 0.33),
                //        Rotation2d.kCCW_90deg,
                //        //1.75,
                //        2,
                //        defaultMoveToConstraints,
                //        false
                //),

                //AutoHelpers.intakeFromLeftNeutralZone(
                //        () -> new Pose2d(
                //                7.1,
                //                4.2,
                //                Rotation2d.kCW_90deg
                //        )
                //).withTimeout(3),

                // avoid scattering balls
                //AutoHelpers.intermediateWaypoint(() -> new Pose2d(
                //        7.5,
                //        trenchShootingPosition.getY() + 0.15,
                //        trenchShootingPosition.getRotation()
                //), defaultMoveToConstraints),

                // move to entrance to trench
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(
                        6.1,
                        startingPositionY,
                        trenchShootingPosition.getRotation()
                ), defaultMoveToConstraints, false),

                // go through trench to shooting position
                AutoHelpers.finalWaypoint(() -> trenchShootingPosition, defaultMoveToConstraints, false),

                // shoot
                Commands.parallel(
                        superintake.intakeShootAlternate(),
                        superstructure.setGoal(Superstructure.Goal.SHOOT),
                        AutoHelpers.finalWaypoint(() -> trenchShootingPosition, defaultMoveToConstraints, true)
                )
        );
    }
}
