// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Util;
import frc.lib.subsystem.Periodic;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

import java.util.*;

import static frc.robot.subsystems.apriltagvision.AprilTagVisionConstants.*;

public class AprilTagVision implements Periodic {
    private final RobotState robotState = RobotState.get();

    private final EnumMap<Camera, CameraData> cameras = Util.createEnumMap(Camera.class,
            Camera.values(), (cam) ->
                    new CameraData(
                            new AprilTagVisionIOInputsAutoLogged(),
                            cam.createIO(),
                            new Alert("AprilTag vision camera " + cam.name() + " is disconnected.", AlertType.kError)
                    ));

    private int[] tagIdFilter = {};


    public Command setTagIdFilter(int[] tagIds) {
        return Commands.runOnce(() -> tagIdFilter = tagIds);
    }

    private final double tagToRobotX = 0.5;
    private final double tagToRobotY = 0.0;
    private final double tagToRobotZ = -0.38;
    private final double tagToRobotYaw = 180.0;
    private double tagToCamQuatW;
    private double tagToCamQuatX;
    private double
            tagToCamQuatY;
    private Rotation3d rotXYZ;
    private double tagToCamQuatZ;
    private double tagToCamTransX;
    private double tagToCamTransY;
    private double tagToCamTransZ;


    private Transform3d tagToCam;
    private Transform3d robotToCam;
    private Transform3d tagToRobot;
    private static AprilTagVision instance;


    public static AprilTagVision get() {
        synchronized (AprilTagVision.class) {
            if (instance == null) {
                instance = new AprilTagVision();
            }
        }

        return instance;
    }

    private AprilTagVision() {
    }

    @Override
    public void periodicBeforeCommands() {
        for (Map.Entry<Camera, CameraData> cam : cameras.entrySet()) {
            Camera metadata = cam.getKey();
            CameraData data = cam.getValue();
            data.io.updateInputs(data.inputs);
            Logger.processInputs("Inputs/AprilTagVision/" + metadata.name(), data.inputs);
            // Update disconnected alert
            data.disconnectedAlert.set(!data.inputs.connected);
        }

        Logger.recordOutput("AprilTagVision/TagIdFilter", tagIdFilter);

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        // Loop over cameras
        for (Map.Entry<Camera, CameraData> cam : cameras.entrySet()) {
            Camera metadata = cam.getKey();
            CameraData data = cam.getValue();

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (var observation : data.inputs.aprilTagObservations) {
                var tagPose = aprilTagLayout.getTagPose(observation.id());
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                } else {
                    Util.error("Couldn't find tag with ID " + observation.id());
                }
            }

            List<SingleTagPoseObservation> singleTagPoseObservations = new LinkedList<>();
            //  robotToCam.clear();
            //rotXYZ.clear();
            for (var observation : data.inputs.bestTargetObservations) {


                tagToCamQuatY = observation.cameraToTarget().inverse().getRotation().getQuaternion().getY();
                tagToCamQuatW = observation.cameraToTarget().inverse().getRotation().getQuaternion().getW();
                tagToCamQuatZ = observation.cameraToTarget().inverse().getRotation().getQuaternion().getZ();
                tagToCamQuatX = observation.cameraToTarget().inverse().getRotation().getQuaternion().getX();
                tagToCamTransX = observation.cameraToTarget().inverse().getTranslation().getX();
                tagToCamTransY = observation.cameraToTarget().inverse().getTranslation().getY();
                tagToCamTransZ = observation.cameraToTarget().inverse().getTranslation().getZ();

                tagToRobot = new Transform3d(new Translation3d(tagToRobotX, tagToRobotY, tagToRobotZ)
                        , new Rotation3d(0, 0, Units.degreesToRadians(tagToRobotYaw)));
                tagToCam = new Transform3d(new Translation3d(tagToCamTransX, tagToCamTransY, tagToCamTransZ)
                        , new Rotation3d(new Quaternion(tagToCamQuatW, tagToCamQuatX, tagToCamQuatY, tagToCamQuatZ)));
                robotToCam = tagToRobot.inverse().plus(tagToCam);
                rotXYZ = new Rotation3d(robotToCam.getRotation().getX(),
                        robotToCam.getRotation().getY(), robotToCam.getRotation().getZ());
                Logger.recordOutput("AprilTagVision/" + cam.getKey() + "robotToCam", robotToCam);
                Logger.recordOutput("AprilTagVision/" + cam.getKey() + "robotToCam/Rotation/X", Units.radiansToDegrees(rotXYZ.getX()));
                Logger.recordOutput("AprilTagVision/" + cam.getKey() + "robotToCam/Rotation/Y", Units.radiansToDegrees(rotXYZ.getY()));
                Logger.recordOutput("AprilTagVision/" + cam.getKey() + "robotToCam/Rotation/Z", Units.radiansToDegrees(rotXYZ.getZ()));


                Optional<Pose3d> tagPoseOptional = aprilTagLayout.getTagPose(observation.tagID());

                if (tagPoseOptional.isEmpty()) {
                    Util.error("Couldn't find tag with ID " + observation.tagID());
                    continue;
                }
                Pose3d tagPose = tagPoseOptional.get();

                double tagDistance = observation.cameraToTarget().getTranslation().getNorm();

                //////////////////////////////// 3d solve ////////////////////////////////
                Transform3d fieldToTarget = new Transform3d(tagPose.getTranslation(), tagPose.getRotation());
                Transform3d fieldToCamera = fieldToTarget.plus(observation.cameraToTarget().inverse());
                Transform3d fieldToRobot = fieldToCamera.plus(metadata.robotToCamera.inverse());
                Pose3d poseEstimate3dSolve = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                //////////////////////////////// Trig ////////////////////////////////
                boolean poseEstimateTrigPresent = false;
                Pose2d poseEstimateTrig = new Pose2d();
                Optional<Rotation2d> headingSampleOptional = robotState.getPoseAtTimestamp(observation.timestamp()).map(Pose2d::getRotation);
                if (headingSampleOptional.isPresent()) {
                    // https://github.com/PhotonVision/photonvision/blob/0ef7c803f91a387a1a95377bf64338509218a240/photon-lib/src/main/java/org/photonvision/PhotonPoseEstimator.java#L496
                    Rotation2d headingSample = headingSampleOptional.get();

                    Translation2d camToTagTranslation = new Translation3d(
                            observation.cameraToTarget().getTranslation().getNorm(),
                            new Rotation3d(
                                    0,
                                    -Math.toRadians(observation.pitch()),
                                    -Math.toRadians(observation.yaw())
                            )
                    )
                            .rotateBy(metadata.robotToCamera.getRotation())
                            .toTranslation2d()
                            .rotateBy(headingSample);

                    Translation2d fieldToCameraTranslation = tagPose
                            .toPose2d()
                            .getTranslation()
                            .plus(camToTagTranslation.unaryMinus());

                    Translation2d camToRobotTranslation = metadata.robotToCamera
                            .getTranslation()
                            .toTranslation2d()
                            .unaryMinus()
                            .rotateBy(headingSample);

                    poseEstimateTrigPresent = true;
                    poseEstimateTrig = new Pose2d(fieldToCameraTranslation.plus(camToRobotTranslation), headingSample);
                }

                singleTagPoseObservations.add(new SingleTagPoseObservation(
                        observation.timestamp(),
                        observation.ambiguity(),
                        observation.tagID(),
                        tagDistance,
                        poseEstimate3dSolve,
                        poseEstimateTrigPresent,
                        poseEstimateTrig
                ));
            }

            List<MultiTagPoseObservation> multiTagPoseObservations = new LinkedList<>();
            for (var observation : data.inputs.multiTagObservations) {
                Transform3d fieldToRobot = observation.fieldToCamera().plus(metadata.robotToCamera.inverse());
                Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                multiTagPoseObservations.add(new MultiTagPoseObservation(
                        observation.timestamp(),
                        observation.ambiguity(),
                        observation.tagCount(),
                        observation.averageTagDistance(),
                        robotPose
                ));
            }

            // Congregate single and multi tag observations
            // and apply trig and tag ID filtering
            List<GenericPoseObservation> genericPoseObservations = new LinkedList<>();
            for (var observation : singleTagPoseObservations) {
                if (tagIdFilter.length > 0) {
                    // If the ID isn't in the filter, skip
                    if (Arrays.stream(tagIdFilter).noneMatch(id -> observation.tagID() == id)) {
                        continue;
                    }
                }

                Pose2d poseEstimate3dSolve2d = observation.poseEstimate3dSolve().toPose2d();
                // If trig is present, distance to tag is small enough, and isn't too different from
                // 3d solve, use trig
                if (observation.poseEstimateTrigPresent() &&
                        observation.tagDistance() <
                                distanceFromTagForTrigMeters &&
                        poseEstimate3dSolve2d.getTranslation()
                                .getDistance(observation.poseEstimateTrig().getTranslation()) < trig3dSolveMaxDiffMeters &&
                        Math.abs(poseEstimate3dSolve2d.getRotation()
                                .minus(observation.poseEstimateTrig().getRotation()).getRadians()) < trig3dSolveMaxDiffRad
                ) {
                    genericPoseObservations.add(new GenericPoseObservation(
                            observation.timestamp(),
                            observation.ambiguity(),
                            1,
                            observation.tagDistance(),
                            new Pose3d(observation.poseEstimateTrig()),
                            linearStdDevBaselineTrigMeters,
                            angularStdDevBaselineTrigRad
                    ));
                } else {
                    genericPoseObservations.add(new GenericPoseObservation(
                            observation.timestamp(),
                            observation.ambiguity(),
                            1,
                            observation.tagDistance(),
                            observation.poseEstimate3dSolve(),
                            linearStdDevBaseline3dSolveMeters,
                            angularStdDevBaseline3dSolveRad
                    ));
                }
            }
            // Due to AdvantageKit constraints, we can't easily log the IDs
            // of tags included in a multitag observation, so only use multitag
            // observations if there is no filter set
            if (tagIdFilter.length == 0) {
                for (var observation : multiTagPoseObservations) {
                    genericPoseObservations.add(new GenericPoseObservation(
                            observation.timestamp(),
                            observation.ambiguity(),
                            observation.tagCount(),
                            observation.averageTagDistance(),
                            observation.poseEstimate(),
                            linearStdDevBaseline3dSolveMeters,
                            angularStdDevBaseline3dSolveRad
                    ));
                }
            }

            // Now that we have congregated best target and multitag observations,
            // we can now filter them and apply them if they are not rejected
            for (var observation : genericPoseObservations) {
                // Check whether to reject pose
                boolean rejectPose =
                        observation.tagCount() == 0 // Must have at least one tag
                                || (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity if only one tag
                                || Math.abs(observation.poseEstimate().getZ()) > maxZError // Must have realistic Z coordinate
                                // Must be within the field boundaries
                                || observation.poseEstimate().getX() < 0.0
                                || observation.poseEstimate().getX() > aprilTagLayout.getFieldLength()
                                || observation.poseEstimate().getY() < 0.0
                                || observation.poseEstimate().getY() > aprilTagLayout.getFieldWidth();

                // Add pose to log
                robotPoses.add(observation.poseEstimate());
                if (rejectPose) {
                    robotPosesRejected.add(observation.poseEstimate());
                } else {
                    robotPosesAccepted.add(observation.poseEstimate());
                }

                // Skip if rejected
                if (rejectPose) {
                    continue;
                }

                // Calculate standard deviations
                double stdDevFactor = Math.pow(observation.averageTagDistance(), metadata.distancePower) / observation.tagCount();
                double linearStdDev = observation.linearStdDevBaseline * stdDevFactor * metadata.stddevMultiplier;
                double angularStdDev = observation.angularStdDevBaseline * stdDevFactor * metadata.stddevMultiplier;

                // Send vision observation
                robotState.addVisionMeasurement(
                        observation.poseEstimate().toPose2d(),
                        observation.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)
                );
            }

            // Log camera data
            String prefix = "AprilTagVision/" + metadata.name() + "/";
            Logger.recordOutput(prefix + "TagPoses", tagPoses.toArray(Pose3d[]::new));
            Logger.recordOutput(prefix + "SingleTagPoseObservations", singleTagPoseObservations.toArray(SingleTagPoseObservation[]::new));
            Logger.recordOutput(prefix + "MultiTagPoseObservations", multiTagPoseObservations.toArray(MultiTagPoseObservation[]::new));
            Logger.recordOutput(prefix + "GenericPoseObservations", genericPoseObservations.toArray(GenericPoseObservation[]::new));
            Logger.recordOutput(prefix + "RobotPoses", robotPoses.toArray(Pose3d[]::new));
            Logger.recordOutput(prefix + "RobotPosesAccepted", robotPosesAccepted.toArray(Pose3d[]::new));
            Logger.recordOutput(prefix + "RobotPosesRejected", robotPosesRejected.toArray(Pose3d[]::new));
            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log summary data
        Logger.recordOutput("AprilTagVision/Summary/TagPoses", allTagPoses.toArray(Pose3d[]::new));
        Logger.recordOutput("AprilTagVision/Summary/RobotPoses", allRobotPoses.toArray(Pose3d[]::new));
        Logger.recordOutput("AprilTagVision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(Pose3d[]::new));
        Logger.recordOutput("AprilTagVision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(Pose3d[]::new));
    }

    @Override
    public void periodicAfterCommands() {
        // Log camera poses for debugging
        var robotPose = new Pose3d(robotState.getPose());
        Logger.recordOutput(
                "AprilTagVision/CameraPoses",
                Arrays.stream(Camera.values())
                        .map(cam -> robotPose.transformBy(cam.robotToCamera))
                        .toArray(Pose3d[]::new)
        );
        Logger.recordOutput("AprilTagVision/ThePose", aprilTagLayout.getTagPose(18).orElseThrow());

    }

    public boolean anyCamerasDisconnected() {
        for (Map.Entry<Camera, CameraData> cam : cameras.entrySet()) {
            CameraData data = cam.getValue();
            return !data.inputs.connected;
        }
        return false;
    }

    private record SingleTagPoseObservation(
            double timestamp,
            double ambiguity,
            int tagID,
            double tagDistance,
            Pose3d poseEstimate3dSolve,
            boolean poseEstimateTrigPresent,
            Pose2d poseEstimateTrig
    ) {
    }

    private record MultiTagPoseObservation(
            double timestamp,
            double ambiguity,
            int tagCount,
            double averageTagDistance,
            Pose3d poseEstimate
    ) {
    }

    private record GenericPoseObservation(
            double timestamp,
            double ambiguity,
            int tagCount,
            double averageTagDistance,
            Pose3d poseEstimate,
            double linearStdDevBaseline,
            double angularStdDevBaseline
    ) {
    }

    private record CameraData(
            AprilTagVisionIOInputsAutoLogged inputs,
            AprilTagVisionIO io,
            Alert disconnectedAlert
    ) {
    }
}
