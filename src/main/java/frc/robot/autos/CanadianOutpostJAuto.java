//package frc.robot.autos;
//
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
//import frc.lib.commands.CommandsExt;
//import frc.robot.RobotState;
//import frc.robot.subsystems.superintake.Superintake;
//import frc.robot.subsystems.superstructure.Superstructure;
//
//import static frc.robot.subsystems.drive.DriveConstants.defaultMoveToConstraints;
//import static frc.robot.subsystems.drive.DriveConstants.shootingConstraints;
//
//public class CanadianOutpostJAuto extends Auto {
//    private static final Superintake superintake = Superintake.get();
//    private static final Superstructure superstructure = Superstructure.get();
//    private static final RobotState robotState = RobotState.get();
//    private static final double startingY = ChoreoTraj.CanadianOutpostJ_FirstPass.initialPoseBlue().getY();
//    private static final Rotation2d startingRotation = ChoreoTraj.CanadianOutpostJ_FirstPass.initialPoseBlue().getRotation();
//    private static final Pose2d preemptiveTrenchEntrance = new Pose2d(3.2, 0.6, Rotation2d.kCCW_90deg);
//    private static final Pose2d trenchEntrance = new Pose2d(3.0, startingY, Rotation2d.kCCW_90deg);
//    private static final Pose2d trenchEntranceNochange = new Pose2d(3.5, startingY, new Rotation2d());
//
//
//    public CanadianOutpostJAuto() {
//        super(
//                new Pose2d(4.35, 0.45, Rotation2d.kCCW_90deg),
//                build()
//        );
//    }
//
//    private static Command build() {
//        return CommandsExt.eagerSequence(
//                // move out of trench
//                AutoHelpers.intermediateWaypoint(() -> new Pose2d(
//                        ChoreoTraj.CanadianOutpostJ_FirstPass.initialPoseBlue().getX() - 0.5,
//                        startingY,
//                        startingRotation
//                ), defaultMoveToConstraints, false),
//
//                // follow collection path
//                Commands.parallel(
//                        AutoHelpers.trajectory(ChoreoTraj.CanadianOutpostJ_FirstPass),
//                        superintake.setGoal(Superintake.Goal.INTAKE).until(() -> true)
//                ),
//
//
//                superintake.setGoal(Superintake.Goal.IDLE
//                ).until(() -> true),
//                AutoHelpers.goOverOutpostSideBump(),
//
//
//                //To stop shoot on move
//                Commands.parallel(
//                        superintake.intakeShootAlternate(),
//                        superstructure.setGoal(Superstructure.Goal.SHOOT),
//                        AutoHelpers.finalWaypoint(robotState::getPose,
//                                defaultMoveToConstraints, true)).withTimeout(4.0),
//
//
//                superstructure.setGoal(Superstructure.Goal.IDLE).until(() -> true),
//
//                // Move to final trench entrance
//                AutoHelpers.intermediateWaypoint(() -> new Pose2d(3.0, startingY + 0.5, Rotation2d.kCCW_90deg),
//                        defaultMoveToConstraints, false),
//
//                AutoHelpers.finalWaypoint(() -> new Pose2d(3.5, startingY, Rotation2d.kCW_90deg), defaultMoveToConstraints, false),
//
//                //Move out for second path
//                AutoHelpers.intermediateWaypoint(() -> new Pose2d(
//                        ChoreoTraj.CanadianOutpostJ_Second.initialPoseBlue().getX(),
//                        startingY,
//                        startingRotation
//                ), defaultMoveToConstraints, false),
//
//                //Follow the second path
//                Commands.parallel(
//                        AutoHelpers.trajectory(ChoreoTraj.CanadianOutpostJ_Second),
//                        superintake.setGoal(Superintake.Goal.INTAKE).until(() -> true)
//                ),
//
//                Commands.parallel(
//                        AutoHelpers.goOverOutpostSideBump(),
//                        superintake.setGoal(Superintake.Goal.IDLE).until(() -> true)
//
//
//                ),
//
//                Commands.parallel(
//                        superintake.intakeShootAlternate(),
//                        superstructure.setGoal(Superstructure.Goal.SHOOT),
//                        AutoHelpers.finalWaypoint(robotState::getPose,
//                                shootingConstraints, true)).withTimeout(4.0)
//
//
//        );
//    }
//}
