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

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.Timer;
import frc955.gamepiecevision.Result;

public class GamePieceVisionIOCoprocessor extends GamePieceVisionIO {
    private final StructSubscriber<Result> resultSubscriber = NetworkTableInstance.getDefault()
            .getStructTopic("/GamePieceVision/Outputs/Result", Result.struct).subscribe(new Result(false, 0.0, new Transform2d[0]));

    private final Debouncer connectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kRising);
    private double lastTimestamp = 0.0;
    private double lastTimestampChanged = 0.0;

    public GamePieceVisionIOCoprocessor() {
    }

    @Override
    public void updateInputs(GamePieceVisionIOInputs inputs) {
        Result result = resultSubscriber.get();

        if (result.timestamp() != lastTimestamp) {
            lastTimestamp = result.timestamp();
            lastTimestampChanged = Timer.getTimestamp();
        }
        inputs.connected = connectedDebouncer.calculate(result.connected()) &&
                Timer.getTimestamp() - lastTimestampChanged < 0.5;

        inputs.timestamp = result.timestamp();
        inputs.clusters = result.clusters();
    }
}
