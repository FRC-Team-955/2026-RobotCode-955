package frc.robot.subsystems.drive.controllers;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.AllianceFlipUtil;
import frc.lib.Util;
import frc.robot.RobotState;
import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

import static frc.robot.subsystems.drive.DriveConstants.choreoFeedbackOmega;
import static frc.robot.subsystems.drive.DriveConstants.choreoFeedbackXY;

public class FollowTrajectoryController {
    private static final RobotState robotState = RobotState.get();

    private final Timer timer = new Timer();
    private final PIDController feedbackX = choreoFeedbackXY.toPID();
    private final PIDController feedbackY = choreoFeedbackXY.toPID();
    private final PIDController feedbackOmega = choreoFeedbackOmega.toPIDWrapRadians();

    public void applyNetworkInputs() {
        if (choreoFeedbackXY.hasChanged()) {
            choreoFeedbackXY.applyPID(feedbackX);
            choreoFeedbackXY.applyPID(feedbackY);
        }

        if (choreoFeedbackOmega.hasChanged()) {
            choreoFeedbackOmega.applyPID(feedbackOmega);
        }
    }

    private @Nullable Trajectory<SwerveSample> trajectory = null;

    public void start(Trajectory<SwerveSample> trajectory) {
        this.trajectory = trajectory;

        timer.restart();

        feedbackX.reset();
        feedbackY.reset();
        feedbackOmega.reset();
    }

    public ChassisSpeeds update() {
        if (trajectory == null) {
            Util.error("Trajectory is null");
            return new ChassisSpeeds();
        }

        Pose2d[] poses = trajectory.getPoses();
        robotState.setTrajectory(Optional.of(poses));
        Logger.recordOutput("Drive/Trajectory", poses);

        var sampleOpt = trajectory.sampleAt(timer.get(), AllianceFlipUtil.shouldFlip());
        if (sampleOpt.isPresent()) {
            SwerveSample sample = sampleOpt.get();

            var currentPose = robotState.getPose();

            robotState.setTrajectorySample(Optional.of(sample.getPose()));
            Logger.recordOutput("Drive/TrajectorySetpoint", sample.getPose());

            return new ChassisSpeeds(
                    sample.vx + feedbackX.calculate(currentPose.getX(), sample.x),
                    sample.vy + feedbackY.calculate(currentPose.getY(), sample.y),
                    sample.omega + feedbackOmega.calculate(currentPose.getRotation().getRadians(), sample.heading)
            );
        } else {
            Util.error("No sample at " + timer.get() + " for trajectory " + trajectory.name());

            robotState.setTrajectorySample(Optional.empty());

            return new ChassisSpeeds();
        }
    }

    public boolean isDone() {
        return trajectory != null && timer.hasElapsed(trajectory.getTotalTime());
    }

    public void stop() {
        trajectory = null;

        robotState.setTrajectory(Optional.empty());
        robotState.setTrajectorySample(Optional.empty());
    }
}
