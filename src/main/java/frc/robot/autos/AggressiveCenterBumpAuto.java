package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.CommandsExt;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;

import static frc.robot.autos.AutoHelpers.intakeConstraints;
import static frc.robot.subsystems.drive.DriveConstants.*;

public class AggressiveCenterBumpAuto extends Auto {
    private static final Pose2d startingAura = new Pose2d(
            FieldConstants.LinesVertical.starting - driveConfig.bumperLengthMeters() / 2.0,
            FieldConstants.LinesHorizontal.center,
            Rotation2d.k180deg);

    public AggressiveCenterBumpAuto() {
        super(startingAura, build());
    }

    private static Command build() {
        return CommandsExt.eagerSequence(
                Commands.race(
                        AutoHelpers.finalWaypoint(
                                () -> new Pose2d(2.0, 5.0, Rotation2d.kZero),
                                defaultMoveToConstraints,
                                true
                        ),
                        Superstructure.getInstance().setGoal(Superstructure.Goal.SHOOT)
                ),
                Superstructure.getInstance().setGoal(Superstructure.Goal.IDLE).until(() -> true),

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
                        Superintake.getInstance().setGoal(Superintake.Goal.INTAKE)
                ),

                Superstructure.getInstance().setGoal(Superstructure.Goal.SHOOT).until(() -> true),

                Commands.race(
                        AutoHelpers.finalWaypoint(
                                () -> new Pose2d(3.3, 5.5, Rotation2d.fromDegrees(135)),
                                shootingConstraints,
                                true
                        ),
                        Superintake.getInstance().intakeShootAlternate()
                ),

                Superstructure.getInstance().setGoal(Superstructure.Goal.IDLE).until(() -> true),
                Superintake.getInstance().setGoal(Superintake.Goal.IDLE).until(() -> true),

                // go over the bump
                AutoHelpers.goOverDepotSideBump(true),

                // follow intake path
                Superintake.getInstance().setGoal(Superintake.Goal.INTAKE).until(() -> true),
                AutoHelpers.trajectory(ChoreoTraj.AggressiveOutpostCenterBump$4, true),
                AutoHelpers.trajectory(ChoreoTraj.AggressiveOutpostCenterBump$5, true),
                //AutoHelpers.intakeOrTrajectory(ChoreoTraj.AggressiveOutpostCenterBump$5, flipY ? leftNeutralZoneBounds : rightNeutralZoneBounds, flipY),
                AutoHelpers.trajectory(ChoreoTraj.AggressiveOutpostCenterBump$6, true),
                Superintake.getInstance().setGoal(Superintake.Goal.IDLE).until(() -> true),

                // go over the bump
                AutoHelpers.goOverDepotSideBump(false),

                Commands.parallel(
                        // shoot
                        Superstructure.getInstance().setGoal(Superstructure.Goal.SHOOT).until(() -> true),
                        Superintake.getInstance().intakeShootAlternate(),
                        Drive.getInstance().stop().withAiming()
                ).withTimeout(10).until(() -> !Superstructure.getInstance().isHasFuel()),

                Superstructure.getInstance().setGoal(Superstructure.Goal.IDLE).until(() -> true),
                Superintake.getInstance().setGoal(Superintake.Goal.IDLE).until(() -> true),

                // go over the bump
                AutoHelpers.goOverDepotSideBump(true),

                Superintake.getInstance().setGoal(Superintake.Goal.INTAKE).until(() -> true),

                AutoHelpers.trajectory(ChoreoTraj.AggressiveOutpostCenterBump$9, true),

                // make sure at entrance of trench
                Superintake.getInstance().setGoal(Superintake.Goal.IDLE).until(() -> true),

                // go over the bump
                AutoHelpers.goOverDepotSideBump(false),

                Commands.parallel(
                        // shoot
                        Superstructure.getInstance().setGoal(Superstructure.Goal.SHOOT).until(() -> true),
                        Superintake.getInstance().intakeShootAlternate(),
                        Drive.getInstance().stop().withAiming()
                )
        );
    }
}
