package frc.robot.subsystems.superintake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controller;
import frc.robot.OperatorDashboard;
import frc.robot.RobotState;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.gamepiecevision.GamePieceVision;
import frc.robot.subsystems.superintake.intakerollers.IntakeRollers;

public abstract class SuperintakeCommand {
    protected final RobotState robotState = RobotState.get();
    protected final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    protected final Controller controller = Controller.get();

    protected final AprilTagVision aprilTagVision = AprilTagVision.get();
    protected final Drive drive = Drive.get();
    protected final GamePieceVision gamePieceVision = GamePieceVision.get();

    protected final Superintake superintake = Superintake.get();
    protected final IntakeRollers intakeRollers = IntakeRollers.get();

    public abstract Command create();

    protected Command rumble() {
        return controller.rumble(0.5, 0.5);
    }
}
