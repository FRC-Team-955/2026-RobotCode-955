package frc.robot.subsystems.questnavvision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.BuildConstants;

public class QuestNavVisionConstants {
    // Robot-center to headset transform. Tune this to your mounting location.
    static final Transform3d robotToQuest =
            new Transform3d(
                    new Translation3d(0.0, 0.0, 0.5),
                    new Rotation3d(0.0, 0.0, 0.0));

    static final double maxZErrorMeters = 0.5;
    static final double linearStdDevMeters = 0.02;
    static final double angularStdDevRad = Units.degreesToRadians(5.0);

    static QuestNavVisionIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL, SIM, REPLAY -> new QuestNavVisionIOQuestNav();
        };
    }

    private QuestNavVisionConstants() {
    }
}

