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

public class OrbitAtHomeDepotAuto extends Auto {
    private static final Superintake superintake = Superintake.get();
    private static final Superstructure superstructure = Superstructure.get();

    private static final double startingPositionY = 7.55;
    private static final Pose2d trenchShootingPosition = new Pose2d(3.6, 7.4, Rotation2d.kCCW_90deg);
    public static final double exitTrenchX = 6.34;

    public OrbitAtHomeDepotAuto() {
        super(
                new Pose2d(3.88, startingPositionY, Rotation2d.kCW_90deg),
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
                ), defaultMoveToConstraints),

                // intake, go to neturalzone
                superintake.setGoal(Superintake.Goal.INTAKE).until(() -> true),
                AutoHelpers.yDistanceInterpolatingWaypoint(
                        new Translation2d(FieldConstants.LinesVertical.center, 7.0),
                        new Translation2d(FieldConstants.LinesVertical.center, 6.4),
                        Rotation2d.kCW_90deg,
                        2,
                        defaultMoveToConstraints
                ),

                //move to netruazone middle
                AutoHelpers.finalWaypoint(() -> new Pose2d(
                        FieldConstants.LinesVertical.center,
                        4.7,
                        Rotation2d.kCW_90deg
                ), AutoHelpers.intakeConstraints),
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),

                // avoid scattering balls
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(
                        FieldConstants.LinesVertical.center - 0.5,
                        trenchShootingPosition.getY(),
                        trenchShootingPosition.getRotation()
                ), defaultMoveToConstraints),

                //move to entrance to trench
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(
                        7.0,
                        7.5,
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
                        exitTrenchX,
                        trenchShootingPosition.getY(),
                        trenchShootingPosition.getRotation()
                ), defaultMoveToConstraints),

                // intake
                AutoHelpers.intakeFromLeftNeutralZone(
                        () -> new Pose2d(
                                7.7,
                                5.9,
                                Rotation2d.kCW_90deg
                        )
                ).withTimeout(3),

                // move to entrance to trench
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(
                        7.0,
                        7.45,
                        trenchShootingPosition.getRotation()
                ), defaultMoveToConstraints),
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),

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
