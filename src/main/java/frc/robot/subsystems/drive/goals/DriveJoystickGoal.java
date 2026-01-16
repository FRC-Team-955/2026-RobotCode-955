package frc.robot.subsystems.drive.goals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Controller;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveGoal;
import frc.robot.subsystems.drive.DriveRequest;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.Supplier;

@RequiredArgsConstructor
public class DriveJoystickGoal extends DriveGoal {
    private static final RobotState robotState = RobotState.get();
    private static final Controller controller = Controller.get();

    @Override
    public DriveRequest getRequest() {
        return DriveRequest.chassisSpeeds(controller.getDriveSetpointRobotRelative(robotState.getRotation()));
    }
}
