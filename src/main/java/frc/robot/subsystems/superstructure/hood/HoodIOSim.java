package frc.robot.subsystems.superstructure.hood;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.lib.motor.MotorIO.MotorIOInputs;
import frc.lib.motor.MotorIOArmSim;
import frc.lib.motor.RequestType;
import frc.lib.network.LoggedTunablePIDF;

import static frc.robot.subsystems.superstructure.hood.HoodConstants.*;

public class HoodIOSim extends HoodIO {
    private final MotorIOArmSim motor;

    public HoodIOSim(double JKgMetersSquared, DCMotor dcMotor) {
        motor = new MotorIOArmSim(
                dcMotor,
                gearRatio,
                JKgMetersSquared,
                Units.inchesToMeters(2),
                minPositionRad,
                maxPositionRad,
                true,
                initialPositionRad,
                0.001,
                gains
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
    public void setVoltageRequest(double volts) {
        motor.setRequest(RequestType.VoltageVolts, volts);
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
        System.out.println("Setting hood neutral mode to " + neutralMode);
    }

    @Override
    public void setCurrentLimit(HoodCurrentLimitMode mode) {
        System.out.println("Setting hood current limit to " + mode);
    }

    @Override
    public void setEncoderPositionToInitial() {
        motor.setEncoderPosition(initialPositionRad);
    }
}
