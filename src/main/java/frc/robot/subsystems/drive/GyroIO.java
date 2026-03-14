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

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.AutoLog;

public class GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public double temperatureCelsius = 0.0;

        public double yawPositionRad = 0.0;
        public Rotation3d orientation = new Rotation3d();

        public double angularVelocityXRadPerSec = 0.0;
        public double angularVelocityYRadPerSec = 0.0;
        public double angularVelocityZRadPerSec = 0.0;

        public double[] odometryYawTimestamps = new double[]{};
        public double[] odometryYawPositionsRad = new double[]{};
    }

    public void updateInputs(GyroIOInputs inputs) {
    }
}
