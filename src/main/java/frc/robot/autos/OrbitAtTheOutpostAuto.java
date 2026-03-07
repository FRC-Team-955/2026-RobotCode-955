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

public class OrbitAtTheOutpostAuto extends Auto {
    private static final Superintake superintake = Superintake.get();
    private static final Superstructure superstructure = Superstructure.get();

    private static final double startingPositionY = 0.7;
    private static final Pose2d trenchShootingPosition = new Pose2d(3.88, 0.7, Rotation2d.kCW_90deg);

    public OrbitAtTheOutpostAuto() {
        super(
                new Pose2d(4.35, startingPositionY, Rotation2d.kCCW_90deg),
                build()
        );
    }

    public static Command build() {
        return CommandsExt.eagerSequence(
                // move out of trench
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(
                        OrbitAtHomeDepotAuto.exitTrenchX,
                        startingPositionY,
                        Rotation2d.kCCW_90deg
                ), defaultMoveToConstraints),

                // intake, go to neturalzone
                superintake.setGoal(Superintake.Goal.INTAKE).until(() -> true),
                AutoHelpers.yDistanceInterpolatingWaypoint(
                        new Translation2d(FieldConstants.LinesVertical.center + OrbitAtHomeDepotAuto.firstPassYOffset, 0.51),
                        new Translation2d(FieldConstants.LinesVertical.center + OrbitAtHomeDepotAuto.firstPassYOffset, 1.81),
                        Rotation2d.kCCW_90deg,
                        2,
                        defaultMoveToConstraints
                ),

                //move to netruazone middle
                AutoHelpers.finalWaypoint(() -> new Pose2d(
                        FieldConstants.LinesVertical.center + OrbitAtHomeDepotAuto.firstPassYOffset,
                        3.51,
                        Rotation2d.kCCW_90deg
                ), AutoHelpers.intakeConstraints),
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),

                // avoid scattering balls
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(
                        FieldConstants.LinesVertical.center - 0.5,
                        trenchShootingPosition.getY() + 0.15,
                        trenchShootingPosition.getRotation()
                ), defaultMoveToConstraints),

                //move to entrance to trench
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(
                        7.0,//6.1 og 6.7 worked
                        startingPositionY, //og trench shooting work
                        trenchShootingPosition.getRotation()
                ), defaultMoveToConstraints),

                // go through trench to shooting position
                AutoHelpers.finalWaypoint(() -> trenchShootingPosition, defaultMoveToConstraints),

                // shoot
                Commands.parallel(
                        superintake.intakeShootAlternate(),
                        superstructure.setGoal(Superstructure.Goal.SHOOT),
                        AutoHelpers.finalWaypoint(() -> trenchShootingPosition, defaultMoveToConstraints.withAiming(true))
                ).withTimeout(4.5),
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),
                superstructure.setGoal(Superstructure.Goal.IDLE).until(() -> true),

                // move out of trench
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(
                        OrbitAtHomeDepotAuto.exitTrenchX,
                        trenchShootingPosition.getY(),
                        trenchShootingPosition.getRotation()
                ), defaultMoveToConstraints),

                // intake
                //AutoHelpers.intakeFromRightNeutralZone(
                //        () -> new Pose2d(
                //                7.1,
                //                3.9,
                //                Rotation2d.kCCW_90deg
                //        )
                //).withTimeout(3),
                superintake.setGoal(Superintake.Goal.INTAKE).until(() -> true),
                AutoHelpers.yDistanceInterpolatingWaypoint(
                        new Translation2d(FieldConstants.LinesVertical.center - OrbitAtHomeDepotAuto.secondPassYOffset, 0.51),
                        new Translation2d(FieldConstants.LinesVertical.center - OrbitAtHomeDepotAuto.secondPassYOffset, 1.81),
                        Rotation2d.kCCW_90deg,
                        2,
                        defaultMoveToConstraints
                ),

                //move to netruazone middle
                AutoHelpers.finalWaypoint(() -> new Pose2d(
                        FieldConstants.LinesVertical.center - OrbitAtHomeDepotAuto.secondPassYOffset,
                        3.51,
                        Rotation2d.kCCW_90deg
                ), AutoHelpers.intakeConstraints),
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),

                // avoid scattering balls
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(
                        7.5,
                        trenchShootingPosition.getY() + 0.15,
                        trenchShootingPosition.getRotation()
                ), defaultMoveToConstraints),

                // move to entrance to trench
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(
                        7.0,//6.1 og 6.7 worked
                        startingPositionY, //og trench shooting work
                        trenchShootingPosition.getRotation()
                ), defaultMoveToConstraints),

                // go through trench to shooting position
                AutoHelpers.finalWaypoint(() -> trenchShootingPosition, defaultMoveToConstraints),

                // shoot
                Commands.parallel(
                        superintake.intakeShootAlternate(),
                        superstructure.setGoal(Superstructure.Goal.SHOOT),
                        AutoHelpers.finalWaypoint(() -> trenchShootingPosition, defaultMoveToConstraints.withAiming(true))
                ).withTimeout(5)
        );
    }
}

