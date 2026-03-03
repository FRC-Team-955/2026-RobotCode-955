package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class Auto {
    public final Pose2d startingPose;
    public final Command command;
}
