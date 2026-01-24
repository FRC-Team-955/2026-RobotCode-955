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

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.lib.LoggedTracer;
import frc.lib.subsystem.Periodic;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.leds.LEDs;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.lang.reflect.Array;
import java.util.List;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private final RobotContainer robotContainer;
    private Command autonomousCommand;
    private double autonomousStart;

    private static List<Periodic> periodics;

    public Robot() {
        @SuppressWarnings("resource")
        Notifier ledsStartupNotifier = LEDs.get().createAndStartStartupNotifier();

        AutoLogOutputManager.addPackage("frc");

        Logger.recordMetadata("* ProjectName", BuildInfo.MAVEN_NAME);
        Logger.recordMetadata("* BuildDate", BuildInfo.BUILD_DATE);
        Logger.recordMetadata("* GitSHA", BuildInfo.GIT_SHA);
        Logger.recordMetadata("* GitDate", BuildInfo.GIT_DATE);
        Logger.recordMetadata("* GitBranch", BuildInfo.GIT_BRANCH);
        switch (BuildInfo.DIRTY) {
            case 0:
                Logger.recordMetadata("* GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("* GitDirty", "Uncommitted changes");
                break;
            default:
                Logger.recordMetadata("* GitDirty", "Unknown");
                break;
        }
        logConstantClass(Constants.class, null);
        logConstantClass(BuildConstants.class, null);

        switch (BuildConstants.mode) {
            case REAL -> {
                Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new NT4Publisher()); // Log to NetworkTables
                // SmartDashboard.putData("PowerDistribution", new PowerDistribution(Constants.pdhId, PowerDistribution.ModuleType.kRev)); // Enables power distribution logging
            }
            case SIM -> {
                Logger.addDataReceiver(new NT4Publisher());
            }
            case REPLAY -> {
                String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
                Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
                if (Constants.Simulation.replayRunAsFastAsPossible) {
                    setUseTiming(false); // Run as fast as possible
                } else {
                    setUseTiming(true); // Run at normal speed
                    Logger.addDataReceiver(new NT4Publisher()); // Log to NetworkTables if we are replaying in real time
                }
            }
        }

        // Start AdvantageKit logger
        Logger.start();

        try {
            // Give some time for the log receiver to start so that printlns from constructors are caught
            // Sleeping isn't the best, but since this just happens on initialization I think it's fine
            Thread.sleep(200);
        } catch (InterruptedException ignored) {
        }

        // Configure brownout voltage
        RobotController.setBrownoutVoltage(6.0);

        // Disable controller disconnection alerts since we have our own alert
        DriverStation.silenceJoystickConnectionWarning(true);

        // No references to RobotContainer/RobotState/any subsystem should be made before this point!
        System.out.println("********** Initializing RobotContainer **********");
        robotContainer = new RobotContainer();

        periodics = List.of(
                // Order matters! Execution order is ascending (that is, the first one listed will execute first)

                // Lots of things depend on controller
                robotContainer.controller,
                // Vision depends on drive
                robotContainer.drive,
                // The rest of the subsystems require vision
                robotContainer.aprilTagVision,
                robotContainer.gamePieceVision,
                // Superintake and superstructure depend on dashboard
                robotContainer.operatorDashboard,
                // Subsystems depend on goals issued by superintake and superstructure
                robotContainer.superintake,
                robotContainer.superstructure,

                // Subsystems - the order of these doesn't matter
                robotContainer.superintake.intakeRollers,
                robotContainer.superintake.intakePivot,
                robotContainer.superstructure.indexer,
                robotContainer.superstructure.flywheel,
                robotContainer.superstructure.hood,

                // Misc
                robotContainer.canLogger,

                // Update the mechanism and LEDs last
                robotContainer.robotMechanism,
                robotContainer.leds
        );

        ledsStartupNotifier.stop();
    }

    private void logConstantClass(Class<?> clazz, String parentName) {
        var parent = (parentName != null ? parentName + "." : "");
        for (var field : clazz.getFields()) {
            var key = parent + clazz.getSimpleName() + "." + field.getName();
            try {
                var value = field.get(null);

                if (value.getClass().isArray()) {
                    for (int i = 0; i < Array.getLength(value); i++) {
                        Logger.recordMetadata(key + "[" + i + "]", Array.get(value, i).toString());
                    }
                } else {
                    Logger.recordMetadata(key, value.toString());
                }
            } catch (Throwable e) {
                Logger.recordMetadata(key, "Unknown");
            }
        }
        for (var subclass : clazz.getClasses()) {
            if (subclass.isEnum())
                continue;

            logConstantClass(subclass, parent + clazz.getSimpleName());
        }
    }

    @Override
    public void robotPeriodic() {
        LoggedTracer.reset();

        for (var periodic : periodics) {
//            System.out.println("periodicBeforeCommands: " + periodic.getClass().getSimpleName());
            periodic.periodicBeforeCommands();
            LoggedTracer.record(periodic.getClass().getSimpleName() + "PeriodicBeforeCommands");
        }

        // Run the command scheduler.
        // Extended subsystems periodic have already been run.
        CommandScheduler.getInstance().run();
        LoggedTracer.record("CommandScheduler");

        if (DriverStation.isAutonomousEnabled()) {
            // We want this to run after the command scheduler,
            // so this can't go in autonomousPeriodic
            if (autonomousCommand != null && !autonomousCommand.isScheduled()) {
                var autonomousEnd = Timer.getTimestamp();
                autonomousCommand = null;
                System.out.printf("********** Auto finished in %.2f seconds **********%n", autonomousEnd - autonomousStart);
            }
        }
        LoggedTracer.record("AutoTimer");

        for (var periodic : periodics) {
//            System.out.println("periodicAfterCommands: " + periodic.getClass().getSimpleName());
            periodic.periodicAfterCommands();
            LoggedTracer.record(periodic.getClass().getSimpleName() + "PeriodicAfterCommands");
        }
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
            autonomousStart = Timer.getTimestamp();
            System.out.println("********** Auto started **********");
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        // We want this to run before the command scheduler, so it goes in autonomousExit
        if (autonomousCommand != null && autonomousCommand.isScheduled()) {
            var autonomousEnd = Timer.getTimestamp();
            autonomousCommand.cancel();
            autonomousCommand = null;
            System.out.printf("********** Auto cancelled in %.2f seconds **********%n", autonomousEnd - autonomousStart);
        }
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
        // In case of replay, don't do sim
        if (BuildConstants.mode == BuildConstants.Mode.REPLAY) return;

        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        SimulatedArena.getInstance().resetFieldForAuto();
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(SimulatedArena.getInstance()::resetFieldForAuto));
        robotContainer.robotState.setPose(ModuleIOSim.driveSimulation.getSimulatedDriveTrainPose());
    }

    @Override
    public void simulationPeriodic() {
        // In case of replay, don't do sim
        if (BuildConstants.mode == BuildConstants.Mode.REPLAY) return;

        SimulatedArena.getInstance().simulationPeriodic();

        Logger.recordOutput("FieldSimulation/RobotPosition", ModuleIOSim.driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Algae",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Algae")
        );
        Logger.recordOutput(
                "FieldSimulation/Coral",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Coral")
        );
    }
}
