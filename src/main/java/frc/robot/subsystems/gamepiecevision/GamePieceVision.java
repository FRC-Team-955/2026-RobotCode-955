package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.commands.CommandsExt;
import frc.lib.subsystem.Periodic;
import frc.robot.OperatorDashboard;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.gamepiecevision.GamePieceVisionConstants.createIO;

public class GamePieceVision implements Periodic {
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    private final GamePieceVisionIO io = createIO();
    private final GamePieceVisionIOInputsAutoLogged inputs = new GamePieceVisionIOInputsAutoLogged();

    private final Alert disconnectedAlert = new Alert("Game piece vision is disconnected.", Alert.AlertType.kError);

    private final Debouncer visibleDebouncer = new Debouncer(0.03);

    private static GamePieceVision instance;

    public static GamePieceVision get() {
        if (instance == null)
            synchronized (GamePieceVision.class) {
                instance = new GamePieceVision();
            }

        return instance;
    }

    private GamePieceVision() {
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/GamePieceVision", inputs);
        // Update disconnected alert
        disconnectedAlert.set(!inputs.connected);

        if (operatorDashboard.forceGamePieceLEDs.hasChanged()) {
            io.setLEDs(operatorDashboard.forceGamePieceLEDs.get());
        }
    }

    public boolean visibleNotDebounced() {
        return inputs.connected && inputs.visible;
    }

    @AutoLogOutput(key = "GamePieceVision/VisibleDebounced")
    public boolean visibleDebounced() {
        return inputs.connected && visibleDebouncer.calculate(inputs.visible);
    }

    public Command waitForGamePiece() {
        return CommandsExt.startEndWaitUntil(
                () -> io.setLEDs(true),
                () -> io.setLEDs(false),
                this::visibleDebounced
        );
    }
}
