package frc.gametest;

import org.junit.jupiter.api.DynamicTest;
import org.junit.jupiter.api.TestFactory;
import org.junit.jupiter.api.parallel.Execution;
import org.junit.jupiter.api.parallel.ExecutionMode;

import java.io.IOException;
import java.util.Collection;
import java.util.List;
import java.util.Scanner;

import static frc.gametest.GameTests.gameTests;
import static org.junit.jupiter.api.Assertions.fail;
import static org.junit.jupiter.api.DynamicTest.dynamicTest;

@SuppressWarnings("NewClassNamingConvention")
class GameTestRunner {
    static final String environmentVariableName = "GAMETEST";

    private static void runTestInSubprocess(String name) throws IOException, InterruptedException {
        boolean isWindows = System.getProperty("os.name").startsWith("Windows");
        var gradleBuilder = new ProcessBuilder(
                isWindows ? "gradlew.bat" : "./gradlew",
                "test",
                "--tests",
                "frc.gametest.GameTestRunner"
        );
        gradleBuilder.environment().put(environmentVariableName, name);
        var gradle = gradleBuilder.start();

        new Thread(() -> {
            Scanner out = new Scanner(gradle.getInputStream());
            while (out.hasNextLine()) {
                System.out.println(out.nextLine());
            }
        }).start();
        new Thread(() -> {
            Scanner err = new Scanner(gradle.getErrorStream());
            while (err.hasNextLine()) {
                System.err.println(err.nextLine());
            }
        }).start();

        if (gradle.waitFor() != 0) {
            fail();
        }
    }

    @TestFactory
    Collection<DynamicTest> createGameTests() {
        String name = System.getenv(environmentVariableName);
        if (System.getenv(environmentVariableName) == null) {
            return gameTests.keySet().stream().map(s -> dynamicTest("[Parent] " + s, () -> runTestInSubprocess(s))).toList();
        }

        var test = gameTests.get(name);
        if (test == null) {
            return List.of(dynamicTest(name, () -> {
                throw new RuntimeException("Unknown game test: " + name);
            }));
        }

        return List.of(dynamicTest("[Child] " + name, test::run));
    }
}
