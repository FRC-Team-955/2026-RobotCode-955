package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.AllianceFlipUtil;
import frc.lib.commands.CommandsExt;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.shooting.ShootingKinematics;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;

import java.util.function.Supplier;

public class DepotAuto {
    private static final RobotState robotState = RobotState.get();
    private static final ShootingKinematics shootingKinematics = ShootingKinematics.get();

    private static final Drive drive = Drive.get();
    private static final Superintake superintake = Superintake.get();
    private static final Superstructure superstructure = Superstructure.get();

    private static Command waypoint(Translation2d translation2d) {
        Supplier<Pose2d> poseSupplier = () -> new Pose2d(
                AllianceFlipUtil.apply(translation2d),
                Rotation2d.fromRadians(shootingKinematics.getShootingParameters().headingRad())
        );
        return drive
                .moveTo(poseSupplier)
                .until(() -> robotState.isAtPoseWithTolerance(
                        poseSupplier.get(),
                        0.05,
                        Units.degreesToRadians(5)
                ));
    }

    public static Command build() {
        return CommandsExt.eagerSequence(
                robotState.setPose(() -> AllianceFlipUtil.apply(new Pose2d(3.5, 5.5, Rotation2d.fromDegrees(-90)))),
                Commands.race(
                        waypoint(FieldConstants.Depot.depotCenter.toTranslation2d()),
                        superintake.setGoal(Superintake.Goal.INTAKE),
                        superstructure.setGoal(Superstructure.Goal.SHOOT)
                ),
                Commands.parallel(
                        superintake.setGoal(Superintake.Goal.INTAKE),
                        superstructure.setGoal(Superstructure.Goal.SHOOT)
                ).withTimeout(3.0),
                waypoint(new Translation2d(3.25, 7.0))
        );
    }
}
