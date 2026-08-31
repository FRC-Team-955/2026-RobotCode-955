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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import frc.lib.HighFrequencySamplingThread;

import java.util.Queue;

import static frc.lib.PhoenixUtil.tryUntilOk;
import static frc.robot.Constants.canivoreBus;

/**
 * IO implementation for Pigeon 2.
 */
public class GyroIOPigeon2 extends GyroIO {
    private final Pigeon2 pigeon;

    private final StatusSignal<Temperature> temperature;
    private final StatusSignal<Angle> yaw;
    private final StatusSignal<AngularVelocity> angularVelocityX;
    private final StatusSignal<AngularVelocity> angularVelocityY;
    private final StatusSignal<AngularVelocity> angularVelocityZ;

    private final Queue<Double> yawTimestampQueue;
    private final Queue<Double> yawPositionQueue;

    private final Debouncer connectedDebouncer = new Debouncer(0.5);

    public GyroIOPigeon2(int canID) {
        pigeon = new Pigeon2(canID, canivoreBus);

        var pigeonConfig = new Pigeon2Configuration();
        tryUntilOk(5, () -> pigeon.getConfigurator().apply(pigeonConfig, 0.25));
        tryUntilOk(5, () -> pigeon.getConfigurator().setYaw(0.0));

        temperature = pigeon.getTemperature();
        yaw = pigeon.getYaw();
        angularVelocityX = pigeon.getAngularVelocityXWorld();
        angularVelocityY = pigeon.getAngularVelocityYWorld();
        angularVelocityZ = pigeon.getAngularVelocityZWorld();

        BaseStatusSignal.setUpdateFrequencyForAll(HighFrequencySamplingThread.frequencyHz, yaw);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                temperature,
                angularVelocityX,
                angularVelocityY,
                angularVelocityZ
        );
        ParentDevice.optimizeBusUtilizationForAll(pigeon);

        yawTimestampQueue = HighFrequencySamplingThread.get().makeTimestampQueue();
        yawPositionQueue = HighFrequencySamplingThread.get().registerPhoenixSignal(pigeon.getYaw());
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        var status = BaseStatusSignal.refreshAll(
                yaw,
                temperature,
                angularVelocityX,
                angularVelocityY,
                angularVelocityZ
        );

        inputs.connected = connectedDebouncer.calculate(status.isOK());
        inputs.temperatureCelsius = temperature.getValueAsDouble();

        inputs.yawPositionRad = Units.degreesToRadians(yaw.getValueAsDouble());
        // getRotation3d uses internal status signals for the quaternion
        inputs.orientation = pigeon.getRotation3d();

        inputs.angularVelocityXRadPerSec = Units.degreesToRadians(angularVelocityX.getValueAsDouble());
        inputs.angularVelocityYRadPerSec = Units.degreesToRadians(angularVelocityY.getValueAsDouble());
        inputs.angularVelocityZRadPerSec = Units.degreesToRadians(angularVelocityZ.getValueAsDouble());

        inputs.odometryYawTimestamps =
                yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositionsRad =
                yawPositionQueue.stream()
                        .mapToDouble(Units::degreesToRadians)
                        .toArray();

        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}
