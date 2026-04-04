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

package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.photonvision.PhotonCamera;

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
        inputs.bestTarget = NetworkTableInstance.getDefault()
                .getStructTopic("GamePieceVision/BestTarget", Transform2d.struct).subscribe(new Transform2d()).get();
        inputs.timestampSeconds = NetworkTableInstance.getDefault()
                .getDoubleTopic("GamePieceVision/timestampSeconds").subscribe(0.0).get();
    }
}
