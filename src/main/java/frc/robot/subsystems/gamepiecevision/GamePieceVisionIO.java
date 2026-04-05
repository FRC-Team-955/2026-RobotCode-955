package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.geometry.Transform2d;
import org.littletonrobotics.junction.AutoLog;

public class GamePieceVisionIO {
    @AutoLog
    public static class GamePieceVisionIOInputs {
        public boolean connected = false;
        public double timestampSeconds = 0.0;
        public Transform2d bestTarget = new Transform2d();
    }

    public void updateInputs(GamePieceVisionIOInputs inputs) {
    }
}