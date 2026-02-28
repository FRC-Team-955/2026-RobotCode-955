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

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.BuildConstants;
import lombok.RequiredArgsConstructor;

import java.util.function.Function;

public class GamePieceVisionConstants {
    static final double horizontalFOVRad = Math.toRadians(160.0);
    static final double camWidth = 640;
    static final double focalLengthPixels = camWidth / (2.0 * Math.tan(horizontalFOVRad / 2.0));
    static final double clusterGroupingDistanceMeters = 0.3;
    static final double diagFOVRad = 2 * Math.atan(Math.tan(horizontalFOVRad / 2) * Math.sqrt(1 + Math.pow(0.75, 2)));
    static final double pixelsToRad = camWidth / horizontalFOVRad;
    static final double minDistanceForSameCoralMeters = 1;
    static final double expireTimeSeconds = 0.5;
    static final double targetMultiplier = 0.5;

    @RequiredArgsConstructor
    enum Camera {
        IntakeCam(
                new Transform3d(
                        Units.inchesToMeters(15.902293), Units.inchesToMeters(11.595038), Units.inchesToMeters(16.533898),
                        // Rotation order matters
                        new Rotation3d(0.0, Units.degreesToRadians(10.0), 0.0)
                                .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(0.0)))
                ),
                (cam) -> switch (BuildConstants.mode) {
                    case REAL -> new GamePieceVisionIOPhotonVision("IntakeCam");
                    case SIM -> new GamePieceVisionIOPhotonVisionSim("IntakeCam", cam.robotToCamera);
                    case REPLAY -> new GamePieceVisionIO();
                }
                // Relatively stable, even at long dist
        ),
        ;

        final Transform3d robotToCamera;
        private final Function<Camera, GamePieceVisionIO> createIO;

        GamePieceVisionIO createIO() {
            return createIO.apply(this);
        }
    }
}
