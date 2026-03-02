package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.AllianceFlipUtil;
import frc.lib.commands.CommandsExt;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;

import static frc.robot.subsystems.drive.DriveConstants.defaultMoveToConstraints;

public class PlanetaryAuto {
    private static final RobotState robotState = RobotState.get();

    private static final Superintake superintake = Superintake.get();
    private static final Superstructure superstructure = Superstructure.get();

    private static final Pose2d trenchShootingPosition = new Pose2d(3.88, 7.3, Rotation2d.kCCW_90deg);
    private static final double exitTrenchX = 6.34;

    public static Command build() {
        return CommandsExt.eagerSequence(
                robotState.setPose(() -> AllianceFlipUtil.apply(new Pose2d(
                        3.88,
                        trenchShootingPosition.getY(),
                        trenchShootingPosition.getRotation()
                ))),

                // move out of trench
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(
                        exitTrenchX,
                        trenchShootingPosition.getY(),
                        trenchShootingPosition.getRotation()
                ), AutoHelpers.trenchConstraints),

                // spin and prepare to intake
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(
                        6.75,
                        6.7,
                        Rotation2d.kCW_90deg
                ), defaultMoveToConstraints),

                // intake, go to neturalzone
                superintake.setGoal(Superintake.Goal.INTAKE).until(() -> true),
                AutoHelpers.yDistanceInterpolatingWaypoint(
                        new Translation2d(FieldConstants.LinesVertical.center, 7.7),
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

                //move to entrance to trench
                AutoHelpers.intermediateWaypoint(() -> new Pose2d(
                        6.1,
                        trenchShootingPosition.getY(),
                        trenchShootingPosition.getRotation()
                ), defaultMoveToConstraints),

                // go through trench to shooting position
                AutoHelpers.intermediateWaypoint(() -> trenchShootingPosition, AutoHelpers.trenchConstraints),

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
                ), AutoHelpers.trenchConstraints),

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
                        6.1,
                        trenchShootingPosition.getY(),
                        trenchShootingPosition.getRotation()
                ), defaultMoveToConstraints),
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),

                // go through trench to shooting position
                AutoHelpers.intermediateWaypoint(() -> trenchShootingPosition, AutoHelpers.trenchConstraints),

                // shoot
                Commands.parallel(
                        superintake.setGoal(Superintake.Goal.INTAKE),
                        superstructure.setGoal(Superstructure.Goal.SHOOT),
                        AutoHelpers.finalWaypoint(() -> trenchShootingPosition, defaultMoveToConstraints.withAiming(true))
                ).withTimeout(5)
        );
    }
}
