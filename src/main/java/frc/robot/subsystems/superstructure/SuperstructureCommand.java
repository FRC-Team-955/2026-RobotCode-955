package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controller;
import frc.robot.OperatorDashboard;
import frc.robot.RobotState;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.gamepiecevision.GamePieceVision;

public abstract class SuperstructureCommand {
    protected final RobotState robotState = RobotState.get();
    protected final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    protected final Controller controller = Controller.get();

    protected final Superstructure superstructure = Superstructure.get();
    protected final AprilTagVision aprilTagVision = AprilTagVision.get();
    protected final Drive drive = Drive.get();
    protected final GamePieceVision gamePieceVision = GamePieceVision.get();

    public abstract Command create();

    protected Command rumble() {
        return controller.rumble(0.5, 0.5);
    }
}
