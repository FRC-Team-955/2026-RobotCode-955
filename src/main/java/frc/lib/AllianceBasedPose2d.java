package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;

import static frc.lib.AllianceFlipUtil.shouldFlip;

public class AllianceBasedPose2d {
    private final Pose2d blue;
    private final Pose2d red;

    public AllianceBasedPose2d(Pose2d blue, Pose2d red) {
        this.blue = blue;
        this.red = red;
    }

    public Pose2d get() {
        return shouldFlip() ? red : blue;
    }
}
