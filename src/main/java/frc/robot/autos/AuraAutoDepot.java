package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.CommandsExt;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;

import static frc.robot.autos.AutoHelpers.intakeConstraints;
import static frc.robot.subsystems.drive.DriveConstants.*;

public class AuraAutoDepot extends Auto {
    private static final Drive drive = Drive.get();
    private static final Superintake superintake = Superintake.get();
    private static final Superstructure superstructure = Superstructure.get();

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
                Commands.deadline(
                        AutoHelpers.finalWaypoint(
                                () -> new Pose2d(2.0, 5.0, Rotation2d.kZero),
                                shootingConstraints,
                                true
                        ),
                        superstructure.setGoal(Superstructure.Goal.SHOOT)
                ),
                superstructure.setGoal(Superstructure.Goal.IDLE).until(() -> true),

                // Move toward depot
                AutoHelpers.intermediateWaypoint(
                        () -> new Pose2d(1.6, 5.0, Rotation2d.kCCW_90deg),
                        intakeConstraints,
                        false
                ),

                // Move to side of depot for easier intaking
                AutoHelpers.finalWaypoint(
                        () -> new Pose2d(0.5, 4.8, Rotation2d.kCCW_90deg),
                        intakeConstraints,
                        false
                ),

                // Intake the Depot
                Commands.race(
                        AutoHelpers.finalWaypoint(
                                () -> new Pose2d(0.5, 6.5, Rotation2d.kCCW_90deg),
                                intakeConstraints,
                                false
                        ),
                        superintake.setGoal(Superintake.Goal.INTAKE)
                ),

                // Stop Intaking
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),

                // Shoot while moving to the outpost
                Commands.race(
                        AutoHelpers.yDistanceInterpolatingWaypoint(
                                new Translation2d(2.5, 5.5),
                                new Translation2d(2.5, FieldConstants.LinesHorizontal.center),
                                Rotation2d.kCW_90deg,
                                1.0,
                                shootingConstraints,
                                true
                        ),
                        superstructure.setGoal(Superstructure.Goal.SHOOT)
                ),

                // Moving backwards to outpost
                Commands.race(
                        AutoHelpers.yDistanceInterpolatingWaypoint(
                                new Translation2d(2.5, 1.0),
                                new Translation2d(1.0, 1.0),
                                new Rotation2d(),
                                2.0,
                                shootingConstraints,
                                true
                        ),
                        superintake.intakeShootAlternate()
                ),

                // Line up to the Outpost
                Commands.race(
                        AutoHelpers.finalWaypoint(
                                () -> new Pose2d(0.5, 0.7, Rotation2d.kCCW_90deg),
                                defaultMoveToConstraints,
                                false
                        ),
                        superintake.setGoal(Superintake.Goal.DEPLOY)
                ),

                // Wait at outpost
                Commands.waitSeconds(2.5),

                // Move back to idle position
                superintake.setGoal(Superintake.Goal.IDLE).until(() -> true),

                // Move closer to the hub to shoot
                Commands.parallel(
                        AutoHelpers.finalWaypoint(
                                () -> new Pose2d(1.5, 1.45, Rotation2d.kCCW_90deg),
                                shootingConstraints,
                                true
                        ),
                        superintake.intakeShootAlternate(),
                        superstructure.setGoal(Superstructure.Goal.SHOOT)
                )
        );
    }
}
