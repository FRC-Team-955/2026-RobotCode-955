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

public class GamePieceVisionConstants {
    /** KEEP SYNCED WITH GAMEPIECEVISION CODE!!! */
    static final Transform3d robotToCamera = new Transform3d(
            Units.inchesToMeters(15.902293), Units.inchesToMeters(11.595038), Units.inchesToMeters(16.533898),
            // Rotation order matters
            new Rotation3d(0.0, Units.degreesToRadians(10.0), 0.0)
                    .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(0.0)))
    );

    static GamePieceVisionIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new GamePieceVisionIOCoprocessor();
            case SIM -> new GamePieceVisionIOCoprocessorSim("IntakeCam");
            case REPLAY -> new GamePieceVisionIO();
        };
    }
}
