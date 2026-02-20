package frc.robot.subsystems.superintake.intakepivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.lib.motor.MotorIO.MotorIOInputs;
import frc.lib.motor.MotorIOArmSim;
import frc.lib.motor.RequestType;
import frc.lib.network.LoggedTunablePIDF;

import static frc.robot.subsystems.superintake.intakepivot.IntakePivotConstants.*;

public class IntakePivotIOSim extends IntakePivotIO {
    private final MotorIOArmSim motor;

    public IntakePivotIOSim(double JKgMetersSquared, DCMotor dcMotor) {
        motor = new MotorIOArmSim(
                dcMotor,
                gearRatio,
                JKgMetersSquared,
                Units.inchesToMeters(10),
                minPositionRad,
                maxPositionRad,
                true,
                Units.degreesToRadians(0),
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
    public void setStopRequest() {
        motor.setRequest(RequestType.VoltageVolts, 0.0);
    }

    @Override
    public void setCurrentLimit(IntakePivotCurrentLimitMode mode) {
        System.out.println("Setting hood current limit to " + mode);
    }
}
