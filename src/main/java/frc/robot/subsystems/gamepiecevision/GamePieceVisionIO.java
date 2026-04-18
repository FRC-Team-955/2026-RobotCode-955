package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.geometry.Transform2d;
import org.littletonrobotics.junction.AutoLog;

public class GamePieceVisionIO {
    @AutoLog
    public static class GamePieceVisionIOInputs {
        public boolean connected = false;
        public double timestamp = 0.0;
        public Transform2d[] clusters = new Transform2d[0];
    }

    public void updateInputs(GamePieceVisionIOInputs inputs) {
    }
}