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
import org.littletonrobotics.junction.AutoLog;

public class AprilTagVisionIO {
    @AutoLog
    public static class AprilTagVisionIOInputs {
        public boolean connected = false;
        public AprilTagTargetObservation[] aprilTagObservations = new AprilTagTargetObservation[0];
        public BestTargetObservation[] bestTargetObservations = new BestTargetObservation[0];
        public MultiTagObservation[] multiTagObservations = new MultiTagObservation[0];
    }

    /**
     * Represents the angle to a simple target, not used for pose estimation.
     */
    public record AprilTagTargetObservation(
            double timestamp,
            int id,
            Rotation2d tx,
            Rotation2d ty
    ) {
    }

    public record BestTargetObservation(
            double timestamp,
            double ambiguity,
            int tagID,
            Transform3d cameraToTarget,
            double pitch,
            double yaw
    ) {}

    public record MultiTagObservation(
            double timestamp,
            double ambiguity,
            int tagCount,
            double averageTagDistance,
            Transform3d fieldToCamera
    ) {}

    public void updateInputs(AprilTagVisionIOInputs inputs) {
    }
}
