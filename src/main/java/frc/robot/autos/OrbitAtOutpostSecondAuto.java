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

public class OrbitAtOutpostSecondAuto extends Auto {
    private static final Superintake superintake = Superintake.get();
    private static final Superstructure superstructure = Superstructure.get();

    //private static final double startingPositionY = 0.45;
    private static final double startingPositionY = 0.52;
    //private static final Pose2d trenchShootingPosition = new Pose2d(3.6, 0.6, Rotation2d.kCW_90deg);
    private static final Pose2d trenchShootingPosition = new Pose2d(3.6, 0.67, Rotation2d.kCW_90deg);

    public OrbitAtOutpostSecondAuto() {
        super(
                new Pose2d(4.35, startingPositionY, Rotation2d.kCCW_90deg),
                build()
        );
    }

    public static Command build() {
        return CommandsExt.eagerSequence(
                // move out of trench
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(
                        OrbitAtDepotSecondAuto.exitTrenchX,
                        startingPositionY,
                        Rotation2d.kCCW_90deg
                ), defaultMoveToConstraints, false),

                // intake, go to neturalzone
                superintake.setGoal(Superintake.Goal.INTAKE).until(() -> true),
                AutoHelpers.yDistanceInterpolatingWaypoint(
                        //new Translation2d(FieldConstants.LinesVertical.center + OrbitAtHomeDepotAuto.firstPassYOffset, 1.0),
                        //new Translation2d(FieldConstants.LinesVertical.center + OrbitAtHomeDepotAuto.firstPassYOffset, 1.6),
                        new Translation2d(FieldConstants.LinesVertical.center, 1.07),
                        new Translation2d(FieldConstants.LinesVertical.center, 1.67),
                        Rotation2d.kCCW_90deg,
                        2,
                        defaultMoveToConstraints,
                        false
                ).withTimeout(3),

                //move to netruazone middle
                AutoHelpers.finalWaypoint(() -> new Pose2d(
                        FieldConstants.LinesVertical.center,
                        //3.3,
                        3.3,
                        Rotation2d.kCCW_90deg
                ), AutoHelpers.intakeConstraints, false).withTimeout(3),
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(FieldConstants.LinesVertical.center -
                                1.0, 3.37,
                                Rotation2d.kCW_90deg),
                        defaultMoveToConstraints,
                        false

                ),


                //// avoid scattering balls
                //AutoHelpers.finalWaypoint(() -> new Pose2d(
                //        FieldConstants.LinesVertical.center - 1.5,
                //        trenchShootingPosition.getY() - 0.15,
                //        trenchShootingPosition.getRotation()
                //), defaultMoveToConstraints, false).withTimeout(3),

                //AutoHelpers.yDistanceInterpolatingWaypoint(
                //        new Translation2d(6.0, 3.5),
                //        new Translation2d(6.0, startingPositionY),
                //        Rotation2d.kCW_90deg,
                //        1.75,
                //        defaultMoveToConstraints,
                //        false
                //),

                //move to entrance to trench
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(
                        6.07,//6.1 og 6.7 worked
                        startingPositionY, //og trench shooting work
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
                        6.5,
                        trenchShootingPosition.getY(),
                        trenchShootingPosition.getRotation()
                ), defaultMoveToConstraints, false),

                // intake
                //AutoHelpers.intakeFromRightNeutralZone(
                //        () -> new Pose2d(
                //                7.1,
                //                3.9,
                //                Rotation2d.kCCW_90deg
                //        )
                //).withTimeout(3),
                Commands.parallel(
                        superintake.setGoal(Superintake.Goal.INTAKE).until(() -> true),
                        AutoHelpers.trajectory(ChoreoTraj.OrbitOutpost)),
                superintake.setGoal(Superintake.Goal.IDLE
                ).until(() -> true),
                AutoHelpers.finalWaypoint(() -> new Pose2d(
                        FieldConstants.LinesVertical.center - 1.5,
                        trenchShootingPosition.getY() + 0.15,
                        trenchShootingPosition.getRotation()
                ), defaultMoveToConstraints, false).withTimeout(3),


                //     AutoHelpers.yDistanceInterpolatingWaypoint(
                //new Translation2d(FieldConstants.LinesVertical.center - OrbitAtHomeDepotAuto.secondPassYOffset, 1.0),
                //        //new Translation2d(FieldConstants.LinesVertical.center - OrbitAtHomeDepotAuto.secondPassYOffset, 3.2),
                //        new Translation2d(FieldConstants.LinesVertical.center - OrbitAtHomeDepotAuto.secondPassYOffset, 0.51),
                //        new Translation2d(FieldConstants.LinesVertical.center - OrbitAtHomeDepotAuto.secondPassYOffset, 1.81),
                //        Rotation2d.kCCW_90deg,
                //        2,
                //        defaultMoveToConstraints,
                //        false
                //),
                //
                ////move to netruazone middle
                //AutoHelpers.intermediateWaypoint(() -> new Pose2d(
                //        FieldConstants.LinesVertical.center - OrbitAtHomeDepotAuto.secondPassYOffset,
                //        4.51,
                //        Rotation2d.kCCW_90deg
                //), AutoHelpers.intakeConstraints, false),
                //
                //
                //superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),
                //
                //AutoHelpers.yDistanceInterpolatingWaypoint(
                //        new Translation2d(6.0, 3.5),
                //        //new Translation2d(6.0, startingPositionY),
                //        new Translation2d(6.0, trenchShootingPosition.getY() - 0.33),
                //        Rotation2d.kCW_90deg,
                //        //1.75,
                //        2,
                //        defaultMoveToConstraints,
                //        false
                //),

                //// avoid scattering balls
                //AutoHelpers.intermediateWaypoint(() -> new Pose2d(
                //        7.5,
                //        trenchShootingPosition.getY() + 0.1,
                //        trenchShootingPosition.getRotation()
                //), defaultMoveToConstraints),

                // move to entrance to trench
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(
                        6.0,//6.1 og 6.7 worked
                        startingPositionY, //og trench shooting work
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


                //Commands.parallel(
                //        AutoHelpers.finalWaypoint( () -> new Pose2d(0.55, 0.85, Rotation2d.k180deg), defaultMoveToConstraints.withAiming(false))
                //).withTimeout
        );
    }
}

