package frc.robot.subsystems.superstructure.hood;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.motor.MotorIO.MotorIOInputs;
import frc.lib.motor.MotorIOSim;
import frc.lib.motor.RequestType;
import frc.lib.network.LoggedTunablePIDF;

import static frc.robot.subsystems.superstructure.hood.HoodConstants.gains;
import static frc.robot.subsystems.superstructure.hood.HoodConstants.gearRatio;

public class HoodIOSim extends HoodIO {
    private final MotorIOSim motor;

    public HoodIOSim(double JKgMetersSquared, DCMotor dcMotor) {
        motor = new MotorIOSim(
                gearRatio,
                JKgMetersSquared,
                dcMotor,
                gains,
                null
        );
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        motor.updateInputs(inputs);
    }

    @Override
    public void setPositionPIDF(LoggedTunablePIDF newGains) {
        motor.setPositionPIDF(newGains);
    }

    @Override
    public void setPositionRequest(double positionRad) {
        motor.setRequest(RequestType.PositionRad, positionRad);
    }

    @Override
    public void setStopRequest() {
        motor.setRequest(RequestType.VoltageVolts, 0.0);
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
        System.out.println("Setting hood neutral mode to " + neutralMode);
    }

    @Override
    public void setCurrentLimit(HoodCurrentLimitMode mode) {
        System.out.println("Setting hood current limit to " + mode);
    }
}
