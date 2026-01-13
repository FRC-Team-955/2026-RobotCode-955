package frc.gametest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.LoggedTracer;
import frc.robot.Robot;
import frc.robot.RobotState;
import org.ironmaple.simulation.SimulatedArena;

import java.lang.reflect.Field;
import java.util.function.BooleanSupplier;

public record GameTest(
        Pose2d startingPose,
        Command command,
        long timeLimitMillis,
        BooleanSupplier assertion
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

        if (!assertion.getAsBoolean()) {
            System.out.println("Assertion failed");
            System.exit(1);
        } else {
            System.out.println("Assertion succeeded");
            System.exit(0);
        }
    }
}
