package frc.robot.subsystems.superstructure.hood;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig;
import frc.lib.motor.MotorIO.MotorIOInputs;
import frc.lib.motor.MotorIOSparkMax;
import frc.lib.motor.RequestType;
import frc.lib.network.LoggedTunablePIDF;

import static frc.robot.subsystems.superstructure.hood.HoodConstants.gains;
import static frc.robot.subsystems.superstructure.hood.HoodConstants.gearRatio;

public class HoodIOSparkMax extends HoodIO {
    private static final int normalCurrentLimitAmps = 40;
    private static final int homingCurrentLimitAmps = 10;

    private final MotorIOSparkMax motor;

    public HoodIOSparkMax(int canID, boolean inverted) {
        motor = new MotorIOSparkMax(
                canID,
                inverted,
                SparkBaseConfig.IdleMode.kBrake,
                normalCurrentLimitAmps,
                gearRatio,
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
        motor.setNeutralMode(neutralMode);
    }

    @Override
    public void setCurrentLimit(HoodCurrentLimitMode mode) {
        System.out.println("Setting hood current limit to " + mode);
        int currentLimitAmps = switch (mode) {
            case NORMAL -> normalCurrentLimitAmps;
            case HOMING -> homingCurrentLimitAmps;
        };
        motor.setCurrentLimit(currentLimitAmps);
    }
}
