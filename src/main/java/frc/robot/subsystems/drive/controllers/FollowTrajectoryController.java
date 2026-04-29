package frc.robot.subsystems.drive.controllers;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.AllianceFlipUtil;
import frc.lib.Util;
import frc.robot.RobotState;
import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.Supplier;

import static frc.robot.subsystems.drive.DriveConstants.choreoFeedbackOmega;
import static frc.robot.subsystems.drive.DriveConstants.choreoFeedbackXY;

public class FollowTrajectoryController {
    private static final RobotState robotState = RobotState.get();

    private final PIDController feedbackX = choreoFeedbackXY.toPID();
    private final PIDController feedbackY = choreoFeedbackXY.toPID();
    private final PIDController feedbackOmega = choreoFeedbackOmega.toPIDWrapRadians();

    private final MoveToController smudgeController = new MoveToController();
    private @Nullable Supplier<Pose2d> smudgeGoalPoseSupplier = null;

    public void applyNetworkInputs() {
        if (choreoFeedbackXY.hasChanged()) {
            choreoFeedbackXY.applyPID(feedbackX);
            choreoFeedbackXY.applyPID(feedbackY);
        }

        if (choreoFeedbackOmega.hasChanged()) {
            choreoFeedbackOmega.applyPID(feedbackOmega);
        }

        smudgeController.applyNetworkInputs();
    }

    private @Nullable Trajectory<SwerveSample> trajectory = null;
    private double minSampleT = 0.0;

    public void start(Trajectory<SwerveSample> trajectory, @Nullable Supplier<Pose2d> smudgeGoalPoseSupplier) {
        this.trajectory = trajectory;
        minSampleT = 0.0;

        feedbackX.reset();
        feedbackY.reset();
        feedbackOmega.reset();

        this.smudgeGoalPoseSupplier = smudgeGoalPoseSupplier;
        if (smudgeGoalPoseSupplier != null) {
            smudgeController.start(smudgeGoalPoseSupplier);
        }
    }

    public ChassisSpeeds update() {
        if (trajectory == null) {
            Util.error("Trajectory is null");
            return new ChassisSpeeds();
        }

        var currentPose = robotState.getPose();

        SwerveSample feedforwardSample = null;
        double minDistanceSq = Double.MAX_VALUE;
        for (var s : trajectory.samples()) {
            if (s.t < minSampleT - 0.1 || s.t > minSampleT + 0.5) continue;
            double dx = s.x - currentPose.getX();
            double dy = s.y - currentPose.getY();
            double distSq = dx * dx + dy * dy;

            if (distSq < minDistanceSq) {
                minDistanceSq = distSq;
                feedforwardSample = s;
            }
        }
        if (feedforwardSample == null) {
            Util.error("No feedforward after " + minSampleT + " for trajectory " + trajectory.name());
            return new ChassisSpeeds();
        }
        minSampleT = feedforwardSample.t;

        var feedbackSample = trajectory.sampleAt(feedforwardSample.t + 0.1, AllianceFlipUtil.shouldFlip())
                .or(() -> trajectory.getFinalSample(AllianceFlipUtil.shouldFlip()))
                .orElse(null);
        if (feedbackSample == null) {
            Util.error("No feedback sample for trajectory " + trajectory.name());
            return new ChassisSpeeds();
        }

        Pose2d[] poses = trajectory.getPoses();
        robotState.setTrajectory(Optional.of(poses));
        robotState.setTrajectorySample(Optional.of(feedforwardSample.getPose()));
        Logger.recordOutput("Drive/Trajectory", poses);
        Logger.recordOutput("Drive/TrajectorySetpoint", feedforwardSample.getPose());

        ChassisSpeeds trajSpeeds = new ChassisSpeeds(
                feedforwardSample.vx + feedbackX.calculate(currentPose.getX(), feedbackSample.x),
                feedforwardSample.vy + feedbackY.calculate(currentPose.getY(), feedbackSample.y),
                feedforwardSample.omega + feedbackOmega.calculate(currentPose.getRotation().getRadians(), feedbackSample.heading)
        );

        ChassisSpeeds smudgeSpeeds = smudgeGoalPoseSupplier != null &&
                smudgeGoalPoseSupplier.get() != null
                ? smudgeController.update()
                : new ChassisSpeeds();

        return trajSpeeds.plus(smudgeSpeeds);
    }

    public boolean isDone() {
        return trajectory != null && minSampleT > trajectory.getTotalTime();
    }
}
