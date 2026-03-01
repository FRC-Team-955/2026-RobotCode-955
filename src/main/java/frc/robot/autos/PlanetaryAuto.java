package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.AllianceFlipUtil;
import frc.lib.commands.CommandsExt;
import frc.robot.RobotState;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;

import java.util.function.Supplier;

import static frc.robot.subsystems.drive.DriveConstants.defaultMoveToConstraints;

public class PlanetaryAuto {
    private static final RobotState robotState = RobotState.get();

    private static final Superintake superintake = Superintake.get();
    private static final Superstructure superstructure = Superstructure.get();

    private static final Pose2d trenchShootingPosition = new Pose2d(3.85, 7.35, Rotation2d.fromDegrees(90));

    public static Command build() {
        Supplier<Command> intake = () -> AutoHelpers.intakeFromLeftNeutralZone(
                        () -> AutoHelpers.yDistanceInterpolation(
                                new Translation2d(7.2, 7.6),
                                new Translation2d(7.5, 4.0),
                                Rotation2d.fromDegrees(-90.0),
                                2
                        )
                ).withTimeout(3);

        return CommandsExt.eagerSequence(
                robotState.setPose(() -> AllianceFlipUtil.apply(new Pose2d(4, 7.4, Rotation2d.fromDegrees(90)))),

                // move out of trench
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(6.25, 7.4, Rotation2d.fromDegrees(-45)), defaultMoveToConstraints),
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(6.75, 6.7, Rotation2d.fromDegrees(-90)), defaultMoveToConstraints),

                // intake, go to neturalzone
                Commands.race(
                        AutoHelpers.yDistanceInterpolatingWaypoint(new Translation2d(4.5, 7.7),
                                new Translation2d(7.45, 6.4)
                                , Rotation2d.fromDegrees(-90),
                                2,
                                defaultMoveToConstraints
                                )),
                        //move to netruazone middle
                        Commands.race(
                                superintake.setGoal(Superintake.Goal.INTAKE),
                                AutoHelpers.finalWaypoint(() -> new Pose2d(7.65, 4.2, Rotation2d.fromDegrees(-90)),
                                AutoHelpers.intakeConstraints)),

                        //move to entrance to trench
                        Commands.race(
                        AutoHelpers.intermediateWaypoint(() -> new Pose2d(6.1, 7.0, trenchShootingPosition.getRotation()), defaultMoveToConstraints),
                        superintake.setGoal(Superintake.Goal.IDLE)
                        ),

                // go through trench to shooting position
                AutoHelpers.finalWaypoint(() -> trenchShootingPosition, defaultMoveToConstraints),

                // shoot
                Commands.parallel(
                        AutoHelpers.finalWaypointWithAimingForever(trenchShootingPosition::getTranslation, defaultMoveToConstraints),
                        superstructure.setGoal(Superstructure.Goal.SHOOT)
                ).withTimeout(5),
                superstructure.setGoal(Superstructure.Goal.IDLE).until(() -> true),

                // move out of trench
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(5.5, 7.4, Rotation2d.fromDegrees(45)), defaultMoveToConstraints),

                // intake
                intake.get(),

                // move to entrance to trench
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(6.1, 7.0, trenchShootingPosition.getRotation()), defaultMoveToConstraints),

                // go through trench to shooting position
                AutoHelpers.finalWaypoint(() -> trenchShootingPosition, defaultMoveToConstraints),

                // shoot
                Commands.parallel(
                        AutoHelpers.finalWaypointWithAimingForever(trenchShootingPosition::getTranslation, defaultMoveToConstraints),
                        superstructure.setGoal(Superstructure.Goal.SHOOT)
                ).withTimeout(5)
        );
    }
}
