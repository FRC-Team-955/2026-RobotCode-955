package frc.robot.subsystems.drive.goals;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.AllianceFlipUtil;
import frc.lib.Util;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveGoal;
import frc.robot.subsystems.drive.DriveRequest;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.drive.DriveConstants.driveConfig;

@RequiredArgsConstructor
public class FollowTrajectoryGoal extends DriveGoal {
    private static final RobotState robotState = RobotState.get();

    private final Trajectory<SwerveSample> trajectory;

    private final Timer timer = new Timer();
    private final PIDController choreoFeedbackX = driveConfig.choreoFeedbackXY().toPID();
    private final PIDController choreoFeedbackY = driveConfig.choreoFeedbackXY().toPID();
    private final PIDController choreoFeedbackOmega = driveConfig.choreoFeedbackOmega().toPIDWrapRadians();

    @Override
    public DriveRequest getRequest() {
        if (!timer.isRunning()) {
            timer.restart();
        }

        Logger.recordOutput("Drive/Trajectory", trajectory.getPoses());

        var sampleOpt = trajectory.sampleAt(timer.get(), AllianceFlipUtil.shouldFlip());
        if (sampleOpt.isPresent()) {
            SwerveSample sample = sampleOpt.get();

            var currentPose = robotState.getPose();

            Logger.recordOutput("Drive/TrajectorySetpoint", sample.getPose());
            return DriveRequest.chassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                    sample.vx + choreoFeedbackX.calculate(currentPose.getX(), sample.x),
                    sample.vy + choreoFeedbackY.calculate(currentPose.getY(), sample.y),
                    sample.omega + choreoFeedbackOmega.calculate(currentPose.getRotation().getRadians(), sample.heading),
                    currentPose.getRotation() // Trajectories are absolute, don't flip
            ));
        } else {
            Util.error("No sample at " + timer.get() + " for trajectory " + trajectory.name());
            return DriveRequest.stop();
        }
    }
}
