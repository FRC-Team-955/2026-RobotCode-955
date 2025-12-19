package frc.lib;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.subsystem.Periodic;
import frc.robot.BuildConstants;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class CANLogger implements Periodic {
    private final Timer roboRIOCANErrorTimer = new Timer();
    private final Timer canivoreErrorTimer = new Timer();
    private final CANBus canivore = new CANBus(Constants.CANivore.busName);

    private final Alert roboRIOCANErrorAlert = new Alert("roboRIO CAN errors detected.", Alert.AlertType.kError);
    private final Alert canivoreErrorAlert = new Alert("CANivore errors detected.", Alert.AlertType.kError);

    private static CANLogger instance;

    public static CANLogger get() {
        if (instance == null)
            synchronized (CANLogger.class) {
                instance = new CANLogger();
            }

        return instance;
    }

    private CANLogger() {
    }

    @Override
    public void periodicAfterCommands() {
        // Check roboRIO CAN status
        var roboRIOCANStatus = RobotController.getCANStatus();
        if (roboRIOCANStatus.transmitErrorCount > 0 || roboRIOCANStatus.receiveErrorCount > 0) {
            roboRIOCANErrorTimer.restart();
        }
        roboRIOCANErrorAlert.set(roboRIOCANErrorTimer.isRunning() && !roboRIOCANErrorTimer.hasElapsed(0.5));

        // Check and log CANivore status
        if (BuildConstants.mode == BuildConstants.Mode.REAL) {
            var canivoreStatus = canivore.getStatus();
            Logger.recordOutput("CANivore/Status", canivoreStatus.Status.getName());
            Logger.recordOutput("CANivore/Utilization", canivoreStatus.BusUtilization);
            Logger.recordOutput("CANivore/OffCount", canivoreStatus.BusOffCount);
            Logger.recordOutput("CANivore/TxFullCount", canivoreStatus.TxFullCount);
            Logger.recordOutput("CANivore/ReceiveErrorCount", canivoreStatus.REC);
            Logger.recordOutput("CANivore/TransmitErrorCount", canivoreStatus.TEC);
            if (!canivoreStatus.Status.isOK()
                    || canivoreStatus.TEC > 0
                    || canivoreStatus.REC > 0
            ) {
                canivoreErrorTimer.restart();
            }
            canivoreErrorAlert.set(canivoreErrorTimer.isRunning() && !canivoreErrorTimer.hasElapsed(0.5));
        }
    }
}
