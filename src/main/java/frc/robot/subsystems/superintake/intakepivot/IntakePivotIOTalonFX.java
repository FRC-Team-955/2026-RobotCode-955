package frc.robot.subsystems.superintake.intakepivot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.motor.MotorIO.MotorIOInputs;
import frc.lib.motor.MotorIOTalonFX;
import frc.lib.motor.RequestType;
import frc.lib.network.LoggedTunablePIDF;

import static frc.robot.subsystems.superintake.intakepivot.IntakePivotConstants.gains;
import static frc.robot.subsystems.superintake.intakepivot.IntakePivotConstants.gearRatio;

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
    public void setCurrentLimit(IntakePivotCurrentLimitMode mode) {
        System.out.println("Setting intake pivot current limit to " + mode);
        int currentLimitAmps = switch (mode) {
            case NORMAL -> normalCurrentLimitAmps;
            case HOMING -> homingCurrentLimitAmps;
        };
        motor.setCurrentLimit(currentLimitAmps);
    }
}
