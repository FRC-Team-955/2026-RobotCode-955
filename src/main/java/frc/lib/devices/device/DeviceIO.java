package frc.lib.devices.device;

/** Device base classes should be abstract so that inheritors are forced to extend all methods. */
public abstract class DeviceIO<Inputs> {
    public abstract void updateInputs(Inputs inputs);
}
