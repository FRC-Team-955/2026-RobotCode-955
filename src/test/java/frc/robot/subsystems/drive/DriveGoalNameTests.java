package frc.robot.subsystems.drive;

import frc.robot.subsystems.drive.goals.FollowTrajectoryGoal;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class DriveGoalNameTests {
    @Test
    void driveGoalTests() {
        assertEquals(new FollowTrajectoryGoal(null).loggableName, "FOLLOW_TRAJECTORY");
    }
}
