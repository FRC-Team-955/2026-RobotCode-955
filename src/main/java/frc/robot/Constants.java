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

package frc.robot;

import com.ctre.phoenix6.CANBus;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
    public static final class Simulation {
        /**
         * If true, replay will run as fast as your computer can go and log to a log file instead of
         * NetworkTables. You will have to open the log file to see anything.
         */
        public static final boolean replayRunAsFastAsPossible = true;
    }

    public static final class CANivore {
        public static final String busName = "rio"; // the canivore is called electrical_problem, but using * is better because it will select any canivore it sees
        public static final boolean isCANFD = switch (BuildConstants.mode) {
            case REAL -> new CANBus(busName).isNetworkFD();
            case SIM, REPLAY -> false;
        };
    }
}
