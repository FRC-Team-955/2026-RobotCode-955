package frc.robot.subsystems.superstructure.turret;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.motor.MotorIO.MotorIOInputs;
import frc.lib.motor.MotorIOArmSim;
import frc.lib.motor.RequestType;
import frc.lib.network.LoggedTunablePIDF;

import static frc.robot.subsystems.superstructure.turret.TurretConstants.*;

public class TurretIOSim extends TurretIO {
    private final MotorIOArmSim motor;

    public TurretIOSim(double JKgMetersSquared, DCMotor dcMotor) {
        // An arm with gravity turned off is just a rotating platform, and unlike the plain motor
        // sim it actually stops at the hard stops
        motor = new MotorIOArmSim(
                dcMotor,
                gearRatio,
                JKgMetersSquared,
                0.3,
                minPositionRad,
                maxPositionRad,
                false,
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
        System.out.println("Setting turret neutral mode to " + neutralMode);
    }

    @Override
    public void setEncoderPosition(double positionRad) {
        motor.setEncoderPosition(positionRad);
    }
}
