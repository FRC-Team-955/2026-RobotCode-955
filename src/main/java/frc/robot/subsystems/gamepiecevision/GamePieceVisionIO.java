package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.geometry.Transform2d;
import org.littletonrobotics.junction.AutoLog;

public class GamePieceVisionIO {
    @AutoLog
    public static class GamePieceVisionIOInputs {
        public boolean connected = false;
        public double timestamp = 0.0;
        public boolean present = false;
        public Transform2d robotToTarget = new Transform2d();
    }

    public void updateInputs(GamePieceVisionIOInputs inputs) {
    }
}