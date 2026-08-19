package frc.robot.subsystems.superstructure.turret;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig;
import frc.lib.motor.MotorIO.MotorIOInputs;
import frc.lib.motor.MotorIOSparkMax;
import frc.lib.motor.RequestType;
import frc.lib.network.LoggedTunablePIDF;

import static frc.robot.subsystems.superstructure.turret.TurretConstants.*;

public class TurretIOSparkMax extends TurretIO {
    private static final int currentLimitAmps = 30;

    private final MotorIOSparkMax motor;

    public TurretIOSparkMax(int canID, boolean inverted) {
        motor = new MotorIOSparkMax(
                canID,
                inverted,
                SparkBaseConfig.IdleMode.kBrake,
                currentLimitAmps,
                gearRatio,
                gains,
                null,
                initialPositionRad
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
        motor.setNeutralMode(neutralMode);
    }

    @Override
    public void setEncoderPosition(double positionRad) {
        motor.setEncoderPosition(positionRad);
    }
}
