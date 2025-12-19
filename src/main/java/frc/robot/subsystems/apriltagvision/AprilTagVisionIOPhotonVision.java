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
import org.photonvision.PhotonCamera;

import java.util.LinkedList;
import java.util.List;

/**
 * IO implementation for real PhotonVision hardware.
 */
public class AprilTagVisionIOPhotonVision extends AprilTagVisionIO {
    protected final PhotonCamera camera;

    public AprilTagVisionIOPhotonVision(String name) {
        camera = new PhotonCamera(name);
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        inputs.connected = camera.isConnected();

        List<AprilTagTargetObservation> aprilTagTargetObservations = new LinkedList<>();
        List<BestTargetObservation> bestTargetObservations = new LinkedList<>();
        List<MultiTagObservation> multiTagObservations = new LinkedList<>();

        for (var result : camera.getAllUnreadResults()) {
            for (var target : result.getTargets()) {
                aprilTagTargetObservations.add(new AprilTagTargetObservation(
                        result.getTimestampSeconds(),
                        target.getFiducialId(),
                        Rotation2d.fromDegrees(target.getYaw()),
                        Rotation2d.fromDegrees(target.getPitch())
                ));
            }

            if (result.hasTargets()) {
                var bestTarget = result.getBestTarget();

                bestTargetObservations.add(new BestTargetObservation(
                        result.getTimestampSeconds(),
                        bestTarget.poseAmbiguity,
                        bestTarget.fiducialId,
                        bestTarget.bestCameraToTarget,
                        bestTarget.getPitch(),
                        bestTarget.getYaw()
                ));
            }

            if (result.multitagResult.isPresent()) {
                var multitagResult = result.multitagResult.get();

                double totalTagDistance = 0.0;
                for (var target : result.targets) {
                    totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
                }

                multiTagObservations.add(new MultiTagObservation(
                        result.getTimestampSeconds(),
                        multitagResult.estimatedPose.ambiguity,
                        multitagResult.fiducialIDsUsed.size(),
                        totalTagDistance / multitagResult.fiducialIDsUsed.size(),
                        multitagResult.estimatedPose.best
                ));
            }
        }

        inputs.aprilTagObservations = aprilTagTargetObservations.toArray(AprilTagTargetObservation[]::new);
        inputs.bestTargetObservations = bestTargetObservations.toArray(BestTargetObservation[]::new);
        inputs.multiTagObservations = multiTagObservations.toArray(MultiTagObservation[]::new);
    }
}
