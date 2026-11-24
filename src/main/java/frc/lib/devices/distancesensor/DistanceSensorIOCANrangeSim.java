package frc.lib.devices.distancesensor;

import com.ctre.phoenix6.sim.CANrangeSimState;
import frc.robot.SimManager;

import java.util.function.DoubleSupplier;

public class DistanceSensorIOCANrangeSim extends DistanceSensorIOCANrange {
    private final CANrangeSimState canRangeSim;

    private final DoubleSupplier distanceSupplier;

    public DistanceSensorIOCANrangeSim(DoubleSupplier distanceSupplier) {
        super(SimManager.getNewCANId());

        canRangeSim = canRange.getSimState();

        this.distanceSupplier = distanceSupplier;
    }

    @Override
    public void updateInputs(DistanceSensorIOInputsAutoLogged inputs) {
        canRangeSim.setDistance(distanceSupplier.getAsDouble());

        super.updateInputs(inputs);
    }
}
