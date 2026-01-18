package frc.lib;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.BuildConstants;

import java.util.EnumMap;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.function.Function;

public class Util {
    private static final double epsilon = 1E-6;

    public static final Executor asyncExecutor = Executors.newFixedThreadPool(4);

    public static boolean shouldFlip() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    public static Rotation2d flipIfNeeded(Rotation2d rotation2d) {
        return shouldFlip()
                ? rotation2d.plus(Rotation2d.kPi)
                : rotation2d;
    }

    public static Pose2d flipIfNeeded(Pose2d pose2d) {
        return shouldFlip()
                ? ChoreoAllianceFlipUtil.flip(pose2d)
                : pose2d;
    }

    public static Translation3d flipIfNeeded(Translation3d translation3d) {
        return shouldFlip()
                ? ChoreoAllianceFlipUtil.flip(translation3d)
                : translation3d;
    }

    private static final Set<String> loggedErrors = new HashSet<>();

    public static void error(String msg) {
        if (BuildConstants.mode == BuildConstants.Mode.SIM) {
            throw new RuntimeException(msg);
        } else {
            if (!loggedErrors.contains(msg)) {
                loggedErrors.add(msg);

                @SuppressWarnings("resource")
                Alert alert = new Alert("Error: " + msg, Alert.AlertType.kError);
                alert.set(true);

                DriverStation.reportError(msg, false);
                System.out.println("Error: " + msg);
            }
        }
    }

    public static boolean epsilonEquals(double a, double b) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(ChassisSpeeds s1, ChassisSpeeds s2) {
        return epsilonEquals(s1.vxMetersPerSecond, s2.vxMetersPerSecond)
                && epsilonEquals(s1.vyMetersPerSecond, s2.vyMetersPerSecond)
                && epsilonEquals(s1.omegaRadiansPerSecond, s2.omegaRadiansPerSecond);
    }

    public static boolean greaterThanEpsilon(double a) {
        return a > epsilon;
    }

    public static int positiveModulus(int input, int modulus) {
        // Ensures returned value is positive
        return ((input % modulus) + modulus) % modulus;
    }

    public static <E extends Enum<E>, V> EnumMap<E, V> createEnumMap(Class<E> clazz, E[] values, Function<E, V> valueSupplier) {
        EnumMap<E, V> map = new EnumMap<>(clazz);
        for (E key : values) {
            map.put(key, valueSupplier.apply(key));
        }
        return map;
    }

    public static String camelCaseToSnakeCase(String input) {
        if (input.length() < 2) {
            return input.toUpperCase();
        }

        StringBuilder output = new StringBuilder();

        // https://docs.rs/heck/
        // Note: we don't implement multiple consecutive uppercase letters for simplicity (it's unnecessary for our use case)
        char last = input.charAt(0);
        output.append(Character.toUpperCase(last));
        for (int i = 1; i < input.length(); i++) {
            char current = input.charAt(i);

            if (Character.isLowerCase(last) && Character.isUpperCase(current)) {
                // New word
                output.append('_');
            }
            output.append(Character.toUpperCase(current));

            last = current;
        }

        return output.toString();
    }
}