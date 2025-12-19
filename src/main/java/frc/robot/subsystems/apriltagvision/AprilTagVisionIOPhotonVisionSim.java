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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.drive.ModuleIOSim;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import java.util.function.Supplier;

import static frc.robot.subsystems.apriltagvision.AprilTagVisionConstants.aprilTagLayout;

/**
 * IO implementation for physics sim using PhotonVision simulator.
 */
public class AprilTagVisionIOPhotonVisionSim extends AprilTagVisionIOPhotonVision {
    private static VisionSystemSim visionSim;

    private static final Supplier<Pose2d> poseSupplier = ModuleIOSim.driveSimulation::getSimulatedDriveTrainPose;
    private final PhotonCameraSim cameraSim;

    public AprilTagVisionIOPhotonVisionSim(String name, Transform3d robotToCamera) {
        super(name);

        // Initialize vision sim
        if (visionSim == null) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(aprilTagLayout);
        }

        // Add sim camera
        var cameraProperties = new SimCameraProperties();
        cameraProperties.setCalibration(1600, 1200, Rotation2d.fromDegrees(95.5));
        cameraProperties.setCalibError(5, 3);
        cameraProperties.setFPS(25);
        cameraProperties.setAvgLatencyMs(45);
        cameraProperties.setLatencyStdDevMs(10);
        cameraSim = new PhotonCameraSim(camera, cameraProperties);
        visionSim.addCamera(cameraSim, robotToCamera);
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        visionSim.update(poseSupplier.get());
        super.updateInputs(inputs);
    }
}
