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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.SimManager;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

public class AprilTagVisionIOPhotonVisionSim extends AprilTagVisionIOPhotonVision {
    public AprilTagVisionIOPhotonVisionSim(String name, Transform3d robotToCamera) {
        super(name);

        // Add sim camera
        var cameraProperties = new SimCameraProperties();
        cameraProperties.setCalibration(1600, 1200, Rotation2d.fromDegrees(70.0));
        cameraProperties.setCalibError(2, 1);
        cameraProperties.setFPS(30);
        cameraProperties.setAvgLatencyMs(45);
        cameraProperties.setLatencyStdDevMs(10);
        PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProperties);
        cameraSim.setMaxSightRange(3.0);
        SimManager.get().visionSystem.addCamera(cameraSim, robotToCamera);
    }
}
