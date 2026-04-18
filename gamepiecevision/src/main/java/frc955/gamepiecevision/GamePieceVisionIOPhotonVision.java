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

package frc955.gamepiecevision;

import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonCamera;

import java.util.LinkedList;
import java.util.List;

/**
 * IO implementation for real PhotonVision hardware.
 */
public class GamePieceVisionIOPhotonVision extends GamePieceVisionIO {
    protected final PhotonCamera camera;

    public GamePieceVisionIOPhotonVision(String name) {
        camera = new PhotonCamera(name);
    }

    @Override
    public void updateInputs(GamePieceVisionIOInputs inputs) {
        inputs.connected = camera.isConnected();

        for (var result : camera.getAllUnreadResults()) {
            List<TargetObservation> targetObservations = new LinkedList<>();

            for (var target : result.getTargets()) {
                targetObservations.add(new TargetObservation(
                        Units.degreesToRadians(target.getYaw()),
                        Units.degreesToRadians(target.getPitch())
                ));
            }

            inputs.timestamp = result.getTimestampSeconds();
            inputs.targetObservations = targetObservations.toArray(TargetObservation[]::new);
        }
    }
}
