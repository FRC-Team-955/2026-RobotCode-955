// MIT License
//
// Copyright (c) 2025-2026 Littleton Robotics
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

package frc.lib;

import edu.wpi.first.wpilibj.RobotController;
import frc.lib.subsystem.Periodic;
import frc.robot.BuildConstants;
import frc.robot.Constants;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Map;

public class EnergyLogger implements Periodic {
    @Getter
    private double totalCurrent = 0.0;
    @Getter
    private double totalPower = 0.0;
    @Getter
    private double totalEnergy = 0.0;

    private final Map<String, Double> subsystemCurrents = new HashMap<>();
    private final Map<String, Double> subsystemPowers = new HashMap<>();
    private final Map<String, Double> subsystemEnergies = new HashMap<>();

    private static EnergyLogger instance;

    public static synchronized EnergyLogger get() {
        if (instance == null) {
            instance = new EnergyLogger();
        }

        return instance;
    }

    private EnergyLogger() {
        if (instance != null) {
            Util.error("Duplicate EnergyLogger created");
        }
    }

    public void reportCurrentUsage(String key, double... amps) {
        if (!BuildConstants.isSimOrReplay) {
            return;
        }

        double totalAmps = 0.0;
        for (double amp : amps) {
            totalAmps += Math.abs(amp);
        }

        double power = totalAmps * RobotController.getBatteryVoltage();
        double energy = power * Constants.loopPeriod;

        totalCurrent += totalAmps;
        totalPower += power;
        totalEnergy += energy;

        subsystemCurrents.put(key, totalAmps);
        subsystemPowers.put(key, power);
        subsystemEnergies.merge(key, energy, Double::sum);

        String[] keys = key.split("/|-");
        if (keys.length < 2) {
            return;
        }
        String subkey = "";
        for (int i = 0; i < keys.length - 1; i++) {
            subkey += keys[i];
            if (i < keys.length - 2) {
                subkey += "/";
            }
            subsystemCurrents.merge(subkey, totalAmps, Double::sum);
            subsystemPowers.merge(subkey, power, Double::sum);
            subsystemEnergies.merge(subkey, energy, Double::sum);
        }
    }

    @Override
    public void periodicAfterCommands() {
        if (!BuildConstants.isSimOrReplay) {
            return;
        }

        reportCurrentUsage("Controls/roboRIO", RobotController.getInputCurrent());
        reportCurrentUsage("Controls/CANcoders", 0.05 * 4);
        reportCurrentUsage("Controls/Pigeon", 0.04);
        reportCurrentUsage("Controls/CANivore", 0.03);
        reportCurrentUsage("Controls/Radio", 0.5);

        for (var entry : subsystemCurrents.entrySet()) {
            Logger.recordOutput("EnergyLogger/Current/" + entry.getKey(), entry.getValue(), "amps");

            subsystemCurrents.put(entry.getKey(), 0.0);
        }
        for (var entry : subsystemPowers.entrySet()) {
            Logger.recordOutput("EnergyLogger/Power/" + entry.getKey(), entry.getValue(), "watts");

            subsystemPowers.put(entry.getKey(), 0.0);
        }
        for (var entry : subsystemEnergies.entrySet()) {
            Logger.recordOutput(
                    "EnergyLogger/Energy/" + entry.getKey(),
                    joulesToWattHours(entry.getValue()),
                    "watt hours"
            );
        }

        Logger.recordOutput("EnergyLogger/TotalCurrent", totalCurrent, "amps");
        Logger.recordOutput("EnergyLogger/TotalPower", totalPower, "watts");
        Logger.recordOutput("EnergyLogger/TotalEnergy", joulesToWattHours(totalEnergy), "watt hours");
        totalCurrent = 0.0;
        totalPower = 0.0;
    }

    private double joulesToWattHours(double joules) {
        return joules / 3600.0;
    }
}

