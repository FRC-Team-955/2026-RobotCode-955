package frc.gametest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.lib.LoggedTracer;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.subsystems.superstructure.SuperstructureIOSim;
import org.ironmaple.simulation.SimulatedArena;

import java.lang.reflect.Field;

public record GameTest(
        Pose2d startingPose,
        Command command,
        long timeLimitMillis,
        Runnable assertion
) {
    public void run() throws InterruptedException, IllegalAccessException, NoSuchFieldException {
        // Set robot state
        RobotState.get().setPose(startingPose);

        // Start robot
        Thread robotRunner = new Thread(() -> {
            RobotBase.startRobot(() -> {
                Robot robot = new Robot();

                // force running as fast as possible
                robot.setUseTiming(false);

                return robot;
            });
        });
        robotRunner.start();

        System.out.println("Waiting for startup to complete");
        // robotPeriodic runs LoggedTracer.reset() every loop
        // that means that once looping starts, startTime will be changed from -1 to the current time
        // use reflection to avoid changing startTime to public
        Field startTime = LoggedTracer.class.getDeclaredField("startTime");
        startTime.setAccessible(true);
        while ((double) startTime.get(null) == -1.0) {
        }
        System.out.println("Startup completion detected");

        // Let things settle before enabling
        Thread.sleep(500);
        // Reset field
        SimulatedArena.getInstance().clearGamePieces();
        // Robot is already enabled by simulationInit, start the command
        command.schedule();

        Thread.sleep(timeLimitMillis);

        assertion.run();
    }
}
