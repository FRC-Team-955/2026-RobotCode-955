package frc.gametest;

import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.LoggedTracer;
import frc.robot.Robot;
import frc.robot.RobotState;
import org.ironmaple.simulation.SimulatedArena;
import org.junit.jupiter.api.DynamicTest;
import org.junit.jupiter.api.TestFactory;

import java.io.IOException;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Scanner;

import static frc.gametest.GameTests.gameTests;
import static org.junit.jupiter.api.Assertions.fail;
import static org.junit.jupiter.api.DynamicTest.dynamicTest;

@SuppressWarnings("NewClassNamingConvention")
class GameTestRunner {
    static final String environmentVariableName = "GAMETEST";

    private static void runGameTest(String name) throws InterruptedException, IOException {
        // Clone the current process, but change the main class to this class

        String javaExecutablePath = ProcessHandle.current().info().command().orElseThrow();
        //System.out.println("Java executable: " + javaExecutablePath);

        ArrayList<String> args = new ArrayList<>();
        args.add(javaExecutablePath);
        for (var arg : ProcessHandle.current().info().arguments().orElseThrow()) {
            if (arg.startsWith("@") || arg.startsWith("-")) {
                args.add(arg);
                //System.out.println("Adding argument: " + arg);
            } else {
                // Arguments are done - use this class as main
                args.add("frc.gametest.GameTestRunner");
                break;
            }
        }

        var gameTestBuilder = new ProcessBuilder(args);
        gameTestBuilder.environment().put(environmentVariableName, name);
        var gameTest = gameTestBuilder.start();
        // If we don't do this, child processes won't stop - see https://stackoverflow.com/q/261125
        Runtime.getRuntime().addShutdownHook(new Thread(gameTest::destroy));

        final StringBuilder output = new StringBuilder();

        new Thread(() -> {
            Scanner out = new Scanner(gameTest.getInputStream());
            while (out.hasNextLine()) {
                // Store in separate var so that we don't lock output variable
                // nextLine might block - we don't want to block while inside the synchronized block
                String nextLine = out.nextLine() + '\n';
                synchronized (output) {
                    output.append(nextLine);
                }
            }
        }).start();
        new Thread(() -> {
            Scanner err = new Scanner(gameTest.getErrorStream());
            while (err.hasNextLine()) {
                // See above comment for while this is not in the synchronized block
                String nextLine = err.nextLine() + '\n';
                synchronized (output) {
                    output.append(nextLine);
                }
            }
        }).start();

        int rc = gameTest.waitFor();
        System.out.println("\n====================================\nBEGIN OUTPUT FOR GAME TEST: " + name + "\n====================================\n");
        System.out.println(output);
        System.out.println("\n====================================\nEND OUTPUT FOR GAME TEST: " + name + "\n====================================\n");
        if (rc != 0) {
            fail();
        }
    }

    @TestFactory
    Collection<DynamicTest> createGameTests() {
        return gameTests.keySet().stream().map(s -> dynamicTest(s, () -> runGameTest(s))).toList();
    }

    public static void main(String[] args) throws NoSuchFieldException, InterruptedException, IllegalAccessException {
        String name = System.getenv(environmentVariableName);
        if (System.getenv(environmentVariableName) == null) {
            throw new RuntimeException("No game test specified");
        }

        var test = gameTests.get(name);
        if (test == null) {
            throw new RuntimeException("Unknown game test: " + name);
        }

        // Set robot state
        RobotState.get().setPose(test.startingPose());

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
        test.command().schedule();

        Thread.sleep(test.timeLimitMillis());

        if (!test.assertion().getAsBoolean()) {
            System.out.println("Assertion failed");
            System.exit(1);
        } else {
            System.out.println("Assertion succeeded");
            System.exit(0);
        }
    }
}
