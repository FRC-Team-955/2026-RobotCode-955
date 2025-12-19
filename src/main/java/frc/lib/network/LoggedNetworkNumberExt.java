package frc.lib.network;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Note: automatically adds <code>/Tuning</code> to key */
public class LoggedNetworkNumberExt extends LoggedNetworkNumber {
    private double lastValue;
    private boolean hasChanged = false;

    public LoggedNetworkNumberExt(String key, double defaultValue) {
        super("/Tuning/" + removeSlash(key), defaultValue);
        lastValue = defaultValue;
    }

    /** Only returns true for one cycle. */
    public boolean hasChanged() {
        return hasChanged;
    }

    @Override
    public void periodic() {
        super.periodic();

        double newValue = get();
        if (newValue != lastValue) {
            hasChanged = true;
        } else if (hasChanged) {
            hasChanged = false;
        }
        lastValue = newValue;
    }
}
