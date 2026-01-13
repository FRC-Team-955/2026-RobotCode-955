package frc.gametest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;

public record GameTest(
        Pose2d startingPose,
        Command command,
        long timeLimitMillis,
        BooleanSupplier assertion
) {
}
