package frc.robot.subsystems.superintake.intakepivot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.motor.MotorIO.MotorIOInputs;
import frc.lib.motor.MotorIOTalonFX;
import frc.lib.motor.RequestType;
import frc.lib.network.LoggedTunablePIDF;

import static frc.robot.subsystems.superintake.intakepivot.IntakePivotConstants.*;

public class IntakePivotIOTalonFX extends IntakePivotIO {
    private static final int normalCurrentLimitAmps = 80;
    private static final int homingCurrentLimitAmps = 20;

    private final MotorIOTalonFX motor;

    public IntakePivotIOTalonFX(int canID, boolean inverted) {
        motor = new MotorIOTalonFX(
                canID,
                inverted,
                NeutralModeValue.Coast,
                normalCurrentLimitAmps,
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
    public void setCurrentLimit(IntakePivotCurrentLimitMode mode) {
        System.out.println("Setting intake pivot current limit to " + mode);
        int currentLimitAmps = switch (mode) {
            case NORMAL -> normalCurrentLimitAmps;
            case HOMING -> homingCurrentLimitAmps;
        };
        motor.setCurrentLimit(currentLimitAmps);
    }

    @Override
    public void setEncoderPositionToInitial() {
        motor.setEncoderPosition(initialPositionRad);
    }
}
