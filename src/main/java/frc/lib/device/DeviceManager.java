package frc.lib.device;

import frc.lib.subsystem.Periodic;
import lombok.Getter;

import java.util.ArrayList;
import java.util.List;

public class DeviceManager implements Periodic {
    @Getter
    private static final DeviceManager instance = new DeviceManager();

    private DeviceManager() {
    }

    private final List<Device<?, ?>> devices = new ArrayList<>();

    /* package-private */ void addDevice(Device<?, ?> device) {
        devices.add(device);
    }

    @Override
    public void periodicBeforeCommands() {
        for (Device<?, ?> device : devices) {
            device.updateAndProcessInputs();
        }
    }
}
