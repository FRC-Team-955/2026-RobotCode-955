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

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.BuildConstants;
import lombok.RequiredArgsConstructor;

import java.util.function.Function;

public class AprilTagVisionConstants {
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

    @RequiredArgsConstructor
    enum Camera {
        // BrainpanCam - ThriftyCam
        BlueCam(
                new Transform3d(
                        Units.inchesToMeters(-11.203173), Units.inchesToMeters(4.211493), Units.inchesToMeters(7.604844),
                        // Rotation order matters
                        new Rotation3d(0.0, Units.degreesToRadians(-15.822126), 0.0)
                                .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(161.0)))
                ),
                (cam) -> switch (BuildConstants.mode) {
                    case REAL -> new AprilTagVisionIOPhotonVision("BlueCam");
                    case SIM -> new AprilTagVisionIOPhotonVisionSim("BlueCam", cam.robotToCamera);
                    case REPLAY -> new AprilTagVisionIO();
                },
                // Relatively stable, even at long distance
                2.0,
                1.0
        ),
        // ShooterCam - OV2311
        GreenCam(
                new Transform3d(
                        Units.inchesToMeters(-12.174301), Units.inchesToMeters(-12.532570), Units.inchesToMeters(7.647165),
                        // Rotation order matters
                        new Rotation3d(0.0, Units.degreesToRadians(-12.260063), 0.0)
                                .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(-168.0)))
                ),
                (cam) -> switch (BuildConstants.mode) {
                    case REAL -> new AprilTagVisionIOPhotonVision("GreenCam");
                    case SIM -> new AprilTagVisionIOPhotonVisionSim("GreenCam", cam.robotToCamera);
                    case REPLAY -> new AprilTagVisionIO();
                },
                // Trust more at close distance, less at long distance
                2.5,
                0.5
        ),
        // HopperCam - OV2311
        YellowCam(
                new Transform3d(
                        Units.inchesToMeters(-5.549223), Units.inchesToMeters(12.125000), Units.inchesToMeters(20.060018),
                        // Rotation order matters
                        new Rotation3d(0.0, Units.degreesToRadians(-35), 0.0)
                ),
                (cam) -> switch (BuildConstants.mode) {
                    case REAL -> new AprilTagVisionIOPhotonVision("YellowCam");
                    case SIM -> new AprilTagVisionIOPhotonVisionSim("YellowCam", cam.robotToCamera);
                    case REPLAY -> new AprilTagVisionIO();
                },
                // Trust more at close distance, less at long distance
                2.0,
                1.0
        ),
        ;

        final Transform3d robotToCamera;
        private final Function<Camera, AprilTagVisionIO> createIO;
        final double distancePower;
        final double stdDevMultiplier;

        AprilTagVisionIO createIO() {
            return createIO.apply(this);
        }
    }
}
