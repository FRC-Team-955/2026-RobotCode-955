package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Controller;
import frc.robot.OperatorDashboard;
import frc.robot.RobotState;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.gamepiecevision.GamePieceVision;

public abstract class SuperstructureCommand {
    protected final RobotState robotState = RobotState.get();
    protected final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    protected final Controller controller = Controller.get();

    protected final Superstructure superstructure = Superstructure.get();
    protected final AprilTagVision aprilTagVision = AprilTagVision.get();
    protected final Drive drive = Drive.get();
    protected final Elevator elevator = Elevator.get();
    protected final EndEffector endEffector = EndEffector.get();
    protected final Funnel funnel = Funnel.get();
    protected final GamePieceVision gamePieceVision = GamePieceVision.get();

    public abstract Command create();

    protected Command waitUntilEndEffectorTriggered() {
        return Commands.waitUntil(superstructure::isEndEffectorTriggered);
    }

    protected Command waitUntilFunnelTriggered() {
        return Commands.waitUntil(superstructure::isFunnelTriggered);
    }

    protected Command waitUntilEndEffectorNotTriggered() {
        return Commands.waitUntil(() -> !superstructure.isEndEffectorTriggered());
    }

    protected Command waitUntilHasNoCoral() {
        return Commands.waitUntil(() -> !superstructure.isHasCoral());
    }

    protected Command rumble() {
        return controller.rumble(0.5, 0.5);
    }

    protected Command shake() {
        return drive.runRobotRelative(() -> Timer.getTimestamp() % 0.25 < 0.125
                ? new ChassisSpeeds(-0.05, -0.05, -0.3)
                : new ChassisSpeeds(0.05, 0.05, 0.3));
    }
}
