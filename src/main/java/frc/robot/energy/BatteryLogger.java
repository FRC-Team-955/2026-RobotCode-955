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

package frc.robot.energy;

import edu.wpi.first.wpilibj.RobotController;
import frc.lib.subsystem.Periodic;
import frc.robot.Constants;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Map;

public class BatteryLogger implements Periodic {
    @Getter
    private double totalCurrent = 0.0;
    @Getter
    private double totalPower = 0.0;
    @Getter
    private double totalEnergy = 0.0;


    @Setter
    private double batteryVoltage = 0.0;
    @Setter
    private double rioCurrent = 0.0;


    private Map<String, Double> subsytemCurrents = new HashMap<>();
    private Map<String, Double> subsytemPowers = new HashMap<>();
    private Map<String, Double> subsytemEnergies = new HashMap<>();


    private static BatteryLogger instance;

    public static synchronized BatteryLogger get() {
        if (instance == null) {
            instance = new BatteryLogger();
        }

        return instance;
    }

    public void reportCurrentUsage(String key, double... amps) {

        double totalAmps = 0.0;
        for (double amp : amps) {
            totalAmps += Math.abs(amp);
        }

        double power = totalAmps * batteryVoltage;
        double energy = power * Constants.loopPeriod;

        totalCurrent += totalAmps;
        totalPower += power;
        totalEnergy += energy;

        subsytemCurrents.put(key, totalAmps);
        subsytemPowers.put(key, power);
        subsytemEnergies.merge(key, energy, Double::sum);

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
            subsytemCurrents.merge(subkey, totalAmps, Double::sum);
            subsytemPowers.merge(subkey, power, Double::sum);
            subsytemEnergies.merge(subkey, energy, Double::sum);
        }
    }

    @Override
    public void periodicAfterCommands() {
        setRioCurrent(RobotController.getInputCurrent());

        setBatteryVoltage(RobotController.getBatteryVoltage());
        Logger.recordOutput("EnergyLogger/BatteryVoltage", batteryVoltage,
                "volts");
        //Logger.recordOutput("EnergyLogger/RioCurrent",
        //        rioCurrent, "amps");


        reportCurrentUsage("EnergyLogger/Controls/roboRIO", rioCurrent);
        reportCurrentUsage("EnergyLogger/Controls/CANcoders", 0.05 * 4);
        reportCurrentUsage("EnergyLogger/Controls/Pigeon", 0.04);
        reportCurrentUsage("EnergyLogger/Controls/CANivore", 0.03);
        reportCurrentUsage("EnergyLogger/Controls/Radio", 0.5);

        for (var entry : subsytemCurrents.entrySet()) {
            Logger.recordOutput("EnergyLogger/Current/" + entry.getKey(), entry.getValue(), "amps");
            subsytemCurrents.put(entry.getKey(), 0.0);
        }
        for (var entry : subsytemPowers.entrySet()) {
            Logger.recordOutput("EnergyLogger/Power/" + entry.getKey(), entry.getValue(), "watts");
            subsytemPowers.put(entry.getKey(), 0.0);
        }
        for (var entry : subsytemEnergies.entrySet()) {
            Logger.recordOutput(
                    "EnergyLogger/Energy/" + entry.getKey(),
                    joulesToWattHours(entry.getValue()),
                    "watt hours");
        }
        totalPower = 0.0;
        totalCurrent = 0.0;
    }

    private double joulesToWattHours(double joules) {
        return joules / 3600.0;
    }
}

