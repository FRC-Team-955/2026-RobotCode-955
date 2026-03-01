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

package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Util;
import frc.lib.subsystem.Periodic;
import frc.robot.controller.Controller;
import frc.robot.shooting.ShootingKinematics;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

/**
 * https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/util/HubShiftUtil.java
 */
public class HubShiftTracker implements Periodic {
    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private static final Controller controller = Controller.get();
    private static final ShootingKinematics shootingKinematics = ShootingKinematics.get();

    public enum ShiftEnum {
        TRANSITION,
        SHIFT1,
        SHIFT2,
        SHIFT3,
        SHIFT4,
        ENDGAME,
        AUTO,
        DISABLED;
    }

    public record ShiftInfo(ShiftEnum currentShift, double elapsedTime, double remainingTime, boolean active) {}

    private static final ShiftEnum[] shiftsEnums = ShiftEnum.values();

    private static final double[] shiftStartTimes = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
    private static final double[] shiftEndTimes = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};

    private static final double minFuelCountDelay = 1.0;
    private static final double maxFuelCountDelay = 2.0;
    private static final double shiftEndFuelCountExtension = 3.0;
    private static final DoubleSupplier approachingActiveFudgeSupplier = () -> -1 * (minFuelCountDelay + shootingKinematics.getShootingParameters().timeOfFlightSeconds().orElse(0.0));
    private static final DoubleSupplier endingActiveFudgeSupplier = () -> shiftEndFuelCountExtension + -1 * (maxFuelCountDelay + shootingKinematics.getShootingParameters().timeOfFlightSeconds().orElse(0.0));

    private static final double autoEndTime = 20.0;
    private static final boolean[] activeSchedule = {true, true, false, true, false, true};
    private static final boolean[] inactiveSchedule = {true, false, true, false, true, true};

    private final Timer shiftTimer = new Timer();

    @Getter
    private ShiftInfo shiftInfo = getShiftedShiftInfo();

    private final Alert gameDataBrokenAlert = new Alert("Game data is broken. Please manually enter who wins in auto.", Alert.AlertType.kError);

    private static HubShiftTracker instance;

    public static synchronized HubShiftTracker get() {
        if (instance == null) {
            instance = new HubShiftTracker();
        }

        return instance;
    }

    private HubShiftTracker() {
        if (instance != null) {
            Util.error("Duplicate HubShiftTracker created");
        }

        new Trigger(() -> shiftInfo.remainingTime() < 3.0)
                .whileTrue(controller.rumble(0.5));
    }

    @Override
    public void periodicBeforeCommands() {
        if (DriverStation.isTeleopEnabled()) {
            if (!shiftTimer.isRunning()) {
                shiftTimer.restart();
            }
        } else {
            shiftTimer.stop();
            shiftTimer.reset();
        }

        shiftInfo = getShiftedShiftInfo();
        Logger.recordOutput("HubShiftTracker/CurrentShift", shiftInfo.currentShift());
        Logger.recordOutput("HubShiftTracker/RemainingTime", shiftInfo.remainingTime());
        Logger.recordOutput("HubShiftTracker/Active", shiftInfo.active());

        ShiftInfo officialShiftInfo = getOfficialShiftInfo();
        Logger.recordOutput("HubShiftTracker/OfficialActive", officialShiftInfo.active());
    }

    private Alliance getFirstActiveAlliance() {
        gameDataBrokenAlert.set(false);

        var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        // Return override value
        if (operatorDashboard.wonAuto.get()) {
            return alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;
        } else if (operatorDashboard.lostAuto.get()) {
            return alliance == Alliance.Blue ? Alliance.Blue : Alliance.Red;
        }

        // Return FMS value
        String message = DriverStation.getGameSpecificMessage();
        if (!message.isEmpty()) {
            char character = message.charAt(0);
            if (character == 'R') {
                return Alliance.Blue;
            } else if (character == 'B') {
                return Alliance.Red;
            }
        }

        // Return default value
        gameDataBrokenAlert.set(true);
        return alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;
    }

    private boolean[] getSchedule() {
        boolean[] currentSchedule;
        Alliance startAlliance = getFirstActiveAlliance();
        currentSchedule = startAlliance == DriverStation.getAlliance().orElse(Alliance.Blue)
                ? activeSchedule
                : inactiveSchedule;
        return currentSchedule;
    }

    private ShiftInfo getShiftInfo(
            boolean[] currentSchedule,
            double[] shiftStartTimes,
            double[] shiftEndTimes
    ) {
        double currentTime = shiftTimer.get();
        double stateTimeElapsed = shiftTimer.get();
        double stateTimeRemaining = 0.0;
        boolean active = false;
        ShiftEnum currentShift = ShiftEnum.DISABLED;

        if (DriverStation.isAutonomousEnabled()) {
            stateTimeElapsed = currentTime;
            stateTimeRemaining = autoEndTime - currentTime;
            active = true;
            currentShift = ShiftEnum.AUTO;
        } else if (DriverStation.isEnabled()) {
            int currentShiftIndex = -1;
            for (int i = 0; i < shiftStartTimes.length; i++) {
                if (currentTime >= shiftStartTimes[i] && currentTime < shiftEndTimes[i]) {
                    currentShiftIndex = i;
                    break;
                }
            }
            if (currentShiftIndex < 0) {
                // After last shift, so assume endgame
                currentShiftIndex = shiftStartTimes.length - 1;
            }

            // Calculate elapsed and remaining time in the current shift, ignoring combined shifts
            stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex];
            stateTimeRemaining = shiftEndTimes[currentShiftIndex] - currentTime;

            // If the state is the same as the last shift, combine the elapsed time
            if (currentShiftIndex > 0) {
                if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex - 1]) {
                    stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex - 1];
                }
            }

            // If the state is the same as the next shift, combine the remaining time
            if (currentShiftIndex < shiftEndTimes.length - 1) {
                if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex + 1]) {
                    stateTimeRemaining = shiftEndTimes[currentShiftIndex + 1] - currentTime;
                }
            }

            active = currentSchedule[currentShiftIndex];
            currentShift = shiftsEnums[currentShiftIndex];
        }
        return new ShiftInfo(currentShift, stateTimeElapsed, stateTimeRemaining, active);
    }

    private ShiftInfo getOfficialShiftInfo() {
        return getShiftInfo(getSchedule(), shiftStartTimes, shiftEndTimes);
    }

    private ShiftInfo getShiftedShiftInfo() {
        boolean[] shiftSchedule = getSchedule();

        double endingActiveFudge = endingActiveFudgeSupplier.getAsDouble();
        Logger.recordOutput("HubShiftTracker/EndingActiveFudge", endingActiveFudge);

        double approachingActiveFudge = approachingActiveFudgeSupplier.getAsDouble();
        Logger.recordOutput("HubShiftTracker/ApproachingActiveFudge", approachingActiveFudge);

        if (shiftSchedule[1]) {
            // Starting active
            double[] shiftedShiftStartTimes = {
                    0.0,
                    10.0,
                    35.0 + endingActiveFudge,
                    60.0 + approachingActiveFudge,
                    85.0 + endingActiveFudge,
                    110.0 + approachingActiveFudge
            };
            double[] shiftedShiftEndTimes = {
                    10.0,
                    35.0 + endingActiveFudge,
                    60.0 + approachingActiveFudge,
                    85.0 + endingActiveFudge,
                    110.0 + approachingActiveFudge,
                    140.0
            };
            return getShiftInfo(shiftSchedule, shiftedShiftStartTimes, shiftedShiftEndTimes);
        } else {
            // Starting inactive
            double[] shiftedShiftStartTimes = {
                    0.0,
                    10.0 + endingActiveFudge,
                    35.0 + approachingActiveFudge,
                    60.0 + endingActiveFudge,
                    85.0 + approachingActiveFudge,
                    110.0
            };
            double[] shiftedShiftEndTimes = {
                    10.0 + endingActiveFudge,
                    35.0 + approachingActiveFudge,
                    60.0 + endingActiveFudge,
                    85.0 + approachingActiveFudge,
                    110.0,
                    140.0
            };
            return getShiftInfo(shiftSchedule, shiftedShiftStartTimes, shiftedShiftEndTimes);
        }
    }
}