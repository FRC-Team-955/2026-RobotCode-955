package frc.lib.devices.device;

import frc.lib.Util;
import frc.lib.devices.motor.Motor;
import frc.lib.subsystem.Periodic;

import java.util.ArrayList;
import java.util.List;

public class DeviceManager implements Periodic {
    private static DeviceManager instance;

    public static synchronized DeviceManager get() {
        if (instance == null) {
            instance = new DeviceManager();
        }

        return instance;
    }

    private DeviceManager() {
        if (instance != null) {
            Util.error("Duplicate DeviceManager created");
        }
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

    public boolean anyDeviceDisconnected() {
        for (Device<?, ?> device : devices) {
            if (!device.isConnected()) {
                return true;
            }
        }
        return false;
    }

    public boolean anyMotorEmergencyStopped() {
        for (Device<?, ?> device : devices) {
            if (device instanceof Motor motor && motor.isEmergencyStopped()) {
                return true;
            }
        }
        return false;
    }

    public boolean anyMotorHasHighTemperature() {
        for (Device<?, ?> device : devices) {
            if (device instanceof Motor motor && motor.hasHighTemperature()) {
                return true;
            }
        }
        return false;
    }
}
