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
    // AprilTag layout

    // Basic filtering thresholds
    static final double maxAmbiguity = 0.3;
    static final double maxZError = 0.2;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    static final double linearStdDevBaselineTrigMeters = 0.1;
    static final double angularStdDevBaselineTrigRad = Units.degreesToRadians(30);
    static final double linearStdDevBaseline3dSolveMeters = 0.3;
    static final double angularStdDevBaseline3dSolveRad = Units.degreesToRadians(60);

    // Distance from a tag for trig estimation to be used
    static final double distanceFromTagForTrigMeters = 1.5;
    // Max difference between 3d solve and trig for trig to be used
    static final double trig3dSolveMaxDiffMeters = 0.2;
    static final double trig3dSolveMaxDiffRad = 0.15;
    static final double horizontalFOVRad = Math.toRadians(70.0);
    static final double camWidth = 640;
    static final double diagFOVRad = 2 * Math.atan(Math.tan(horizontalFOVRad / 2) * Math.sqrt(1 + Math.pow(0.75, 2)));
    static final double pixelsToRad = camWidth / horizontalFOVRad;
    public static final double fuelDiameterMeters = 0.15;
    static final double minDistanceForSameCoralMeters = 1;
    static final double expireTimeSeconds = 0.5;

    static final double coralHeightMeters = Units.inchesToMeters(4.25);


    @RequiredArgsConstructor
    enum Camera {
        GPVCam(

                new Transform3d(
                        Units.inchesToMeters(3.770), Units.inchesToMeters(-12.406), Units.inchesToMeters(20.485),
                        // Rotation order matters
                        new Rotation3d(0.0, Units.degreesToRadians(6.963), 0.0)
                                .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(10.0)))
                ),
                (cam) -> switch (BuildConstants.mode) {
                    case REAL -> new GamePieceVisionIOPhotonVision("GPVCam");
                    case SIM -> new GamePieceVisionIOPhotonVisionSim("GPVCam", cam.robotToCamera);
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
