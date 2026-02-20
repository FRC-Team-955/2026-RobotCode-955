package frc.robot.subsystems.drive.goals;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.PIDF;
import frc.robot.Controller;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveGoal;
import frc.robot.subsystems.drive.DriveRequest;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.Supplier;

@RequiredArgsConstructor
public class DriveJoystickWithAssistGoal extends DriveGoal {
    private static final RobotState robotState = RobotState.get();
    private static final Controller controller = Controller.get();

    private static final PIDController assistPIDY = PIDF.ofPD(4.5, 0.0).toPID();
    private final Supplier<Optional<Pose2d>> assistPoseSupplier;

    @Override
    public DriveRequest getRequest() {
        var optionalAssistPose = assistPoseSupplier.get();
        if (optionalAssistPose.isPresent()) {
            // Mark assist pose as present
            Logger.recordOutput("Drive/Assist/Present", true);
            Pose2d assistPose = optionalAssistPose.get();

            if (controller.shouldAssist(robotState.getPose(), assistPose)) {
                Logger.recordOutput("Drive/Assist/Running", true);
                return DriveRequest.chassisSpeeds(getAssisted(assistPose));
            }
        } else {
            Logger.recordOutput("Drive/Assist/Present", false);
        }

        Logger.recordOutput("Drive/Assist/Running", false);
        return DriveRequest.chassisSpeeds(controller.getDriveSetpointRobotRelative(robotState.getRotation()));
    }

    private static ChassisSpeeds getAssisted(Pose2d assistPose) {
        var currentPose = robotState.getPose();
//        double assistX = 0;
//        //   double assistY = 0;
//        double assistOmega = 0;
        // TODO: need to reset the PIDs when assist starts
        // TODO: log setpoint
//        double assistX = moveToLinearX.calculate(
//                currentPose.getX(),
//                assistPose.getX()
//        ) + moveToLinearX.getSetpoint().velocity;
//        assistX *= linearMagnitude; // Limit to the driver's overall linear speed
//
        // double assistY = new Transform2d(currentPose, assistPose).getY() * assistPIDY.getP();
        double assistY = assistPIDY.calculate(new Transform2d(currentPose, assistPose).getY(), 0);

//         controller.getDriveSetpointRobotRelative(currentPose.getRotation())
        assistY = assistY >= controller.getDriveLinearMagnitude() ? controller.getDriveLinearMagnitude() : assistY; // Limit to the driver's overall linear speed
//
//        double assistOmega = moveToAngular.calculate(
//                currentPose.getRotation().getRadians(),
//                assistPose.getRotation().getRadians()
//        ) + moveToAngular.getSetpoint().velocity;
//        // If we are driving fast and not rotating, need fast rotation assist, so limit to driver's overall linear speed
//        // Otherwise, limit to driver omega speed
//        assistOmega *= Math.max(omegaMagnitude, linearMagnitude);
        double vx = controller.getDriveSetpointRobotRelative(currentPose.getRotation()).vxMetersPerSecond;
        double omega = controller.getDriveSetpointRobotRelative(currentPose.getRotation()).omegaRadiansPerSecond;
        return ChassisSpeeds.fromRobotRelativeSpeeds(
                vx,
                assistY,
                omega,
                currentPose.getRotation() // Move to is absolute, don't flip
        );
    }
}
