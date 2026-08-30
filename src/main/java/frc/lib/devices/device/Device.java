package frc.lib.devices.device;

import edu.wpi.first.wpilibj.Alert;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public abstract class Device<IO extends DeviceIO<Inputs>, Inputs extends LoggableInputs> {
    protected final String name;
    protected final IO io;
    protected final Inputs inputs;

    private final Alert disconnectedAlert;

    protected Device(String name, IO io, Inputs inputs) {
        this.name = name;
        this.io = io;
        this.inputs = inputs;

        disconnectedAlert = new Alert(name + " is disconnected.", Alert.AlertType.kError);

        DeviceManager.getInstance().addDevice(this);
    }

    protected void updateAndProcessInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/" + name, inputs);

        disconnectedAlert.set(!isConnected());
    }

    public abstract boolean isConnected();
}
