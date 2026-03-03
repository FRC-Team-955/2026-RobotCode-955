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

public class PlanetaryRightAuto extends Auto {
    private static final Superintake superintake = Superintake.get();
    private static final Superstructure superstructure = Superstructure.get();

    private static final Pose2d trenchShootingPosition = new Pose2d(3.88, 0.7, Rotation2d.fromDegrees(-90));
    private static final double exitTrenchX = 6.25;

    public PlanetaryRightAuto() {
        super(
                new Pose2d(3.88, trenchShootingPosition.getY(), trenchShootingPosition.getRotation()),
                build()
        );
    }

    public static Command build() {
        return CommandsExt.eagerSequence(
                // move out of trench
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(
                        exitTrenchX,
                        trenchShootingPosition.getY(),
                        trenchShootingPosition.getRotation()
                ), defaultMoveToConstraints),

                // spin and prepare to intake
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(
                        6.75,
                        1.51,
                        Rotation2d.fromDegrees(90)
                ), defaultMoveToConstraints),

                // intake, go to neturalzone
                superintake.setGoal(Superintake.Goal.INTAKE).until(() -> true),
                AutoHelpers.yDistanceInterpolatingWaypoint(
                        new Translation2d(FieldConstants.LinesVertical.center, 0.51),
                        new Translation2d(FieldConstants.LinesVertical.center, 1.81),
                        Rotation2d.fromDegrees(90),
                        2,
                        defaultMoveToConstraints
                ),

                //move to netruazone middle
                AutoHelpers.finalWaypoint(() -> new Pose2d(
                        FieldConstants.LinesVertical.center,
                        3.51,
                        Rotation2d.fromDegrees(90)
                ), AutoHelpers.intakeConstraints),
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),

                //move to entrance to trench
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(
                        6.1,
                        trenchShootingPosition.getY(),
                        trenchShootingPosition.getRotation()
                ), defaultMoveToConstraints),

                // go through trench to shooting position
                AutoHelpers.intermediateWaypoint(() -> trenchShootingPosition, defaultMoveToConstraints),

                // shoot
                Commands.parallel(
                        superintake.setGoal(Superintake.Goal.INTAKE),
                        superstructure.setGoal(Superstructure.Goal.SHOOT),
                        AutoHelpers.finalWaypoint(() -> trenchShootingPosition, defaultMoveToConstraints.withAiming(true))
                ).withTimeout(5),
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),
                superstructure.setGoal(Superstructure.Goal.IDLE).until(() -> true),

                // move out of trench
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(
                        exitTrenchX,
                        trenchShootingPosition.getY(),
                        trenchShootingPosition.getRotation()
                ), defaultMoveToConstraints),

                // intake
                AutoHelpers.intakeFromRightNeutralZone(
                        () -> AutoHelpers.yDistanceInterpolation(
                                new Translation2d(7.5, 0.61),
                                new Translation2d(7.5, 4.21),
                                Rotation2d.fromDegrees(90.0),
                                2
                        )
                ).withTimeout(3),

                // move to entrance to trench
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(
                        6.1,
                        trenchShootingPosition.getY(),
                        trenchShootingPosition.getRotation()
                ), defaultMoveToConstraints),
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),

                // go through trench to shooting position
                AutoHelpers.intermediateWaypoint(() -> trenchShootingPosition, defaultMoveToConstraints),

                // shoot
                Commands.parallel(
                        superintake.setGoal(Superintake.Goal.INTAKE),
                        superstructure.setGoal(Superstructure.Goal.SHOOT),
                        AutoHelpers.finalWaypoint(() -> trenchShootingPosition, defaultMoveToConstraints.withAiming(true))
                ).withTimeout(5)
        );
    }
}

