package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.AllianceFlipUtil;
import frc.lib.commands.CommandsExt;
import frc.robot.RobotState;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;

import java.util.function.Supplier;

import static frc.robot.subsystems.drive.DriveConstants.defaultMoveToConstraints;

public class BigYahuAuto {
    private static final RobotState robotState = RobotState.get();

    private static final Superintake superintake = Superintake.get();
    private static final Superstructure superstructure = Superstructure.get();

    private static final Pose2d trenchShootingPosition = new Pose2d(3.85, 7.35, Rotation2d.fromDegrees(90));

    public static Command build() {
        Supplier<Command> intake = () -> Commands.parallel(
                AutoHelpers.intakeFromLeftNeutralZone(
                        () -> AutoHelpers.yDistanceInterpolation(
                                new Translation2d(7.4, 7.4),
                                new Translation2d(7.4, 4.5),
                                Rotation2d.fromDegrees(-45.0 * Timer.getTimestamp()),
                                2
                        )
                ),
                superintake.setGoal(Superintake.Goal.INTAKE).asProxy()
        ).withTimeout(3);

        return CommandsExt.eagerSequence(
                robotState.setPose(() -> AllianceFlipUtil.apply(new Pose2d(4, 7.4, Rotation2d.fromDegrees(-90)))),

                // move out of trench
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(5, 7.4, Rotation2d.fromDegrees(-90)), defaultMoveToConstraints),

                // intake
                intake.get(),

                // move to entrance to trench
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(6.1, 7.3, trenchShootingPosition.getRotation()), defaultMoveToConstraints),

                // go through trench to shooting position
                AutoHelpers.finalWaypoint(() -> trenchShootingPosition, defaultMoveToConstraints),

                // shoot
                Commands.parallel(
                        AutoHelpers.finalWaypointWithAimingForever(trenchShootingPosition::getTranslation, defaultMoveToConstraints),
                        superstructure.setGoal(Superstructure.Goal.SHOOT).asProxy()
                ).withTimeout(4),

                // move out of trench
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(5, 7.4, Rotation2d.fromDegrees(90)), defaultMoveToConstraints),

                // intake
                intake.get(),

                // move to entrance to trench
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(6.1, 7.3, trenchShootingPosition.getRotation()), defaultMoveToConstraints),

                // go through trench to shooting position
                AutoHelpers.finalWaypoint(() -> trenchShootingPosition, defaultMoveToConstraints),

                // shoot
                Commands.parallel(
                        AutoHelpers.finalWaypointWithAimingForever(trenchShootingPosition::getTranslation, defaultMoveToConstraints),
                        superstructure.setGoal(Superstructure.Goal.SHOOT).asProxy()
                ).withTimeout(4)
        );
    }
}
