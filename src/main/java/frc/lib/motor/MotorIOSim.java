package frc.lib.motor;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.network.LoggedTunablePIDF;
import frc.robot.Constants;

public class MotorIOSim extends MotorIO {
    private final DCMotorSim motorSim;
    private PIDController positionPid;
    private PIDController velocityPid;
    private SimpleMotorFeedforward velocityFeedforward;

    private double appliedVolts;
    private boolean closedLoop = true;
    private boolean positionControl = false;
    private double ffVolts;

    // If using physical values, JKgMetersSquared is the moment of inertia J of the flywheel
    public MotorIOSim(
            double gearRatio,
            double JKgMetersSquared,
            DCMotor motor,
            LoggedTunablePIDF positionGains,
            LoggedTunablePIDF velocityGains,
            double initialPositionRad
    ) {
        motorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(motor, JKgMetersSquared, gearRatio),
                motor,
                0.004,
                0.0
        );
        motorSim.setAngle(initialPositionRad);

        velocityFeedforward = velocityGains != null ? velocityGains.toSimpleFF() : new SimpleMotorFeedforward(0, 0);
        positionPid = positionGains != null ? positionGains.toPID() : new PIDController(0, 0, 0);
        velocityPid = velocityGains != null ? velocityGains.toPID() : new PIDController(0, 0, 0);
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        if (closedLoop) {
            if (positionControl) {
                appliedVolts = positionPid.calculate(motorSim.getAngularPositionRad());
            } else {
                appliedVolts = velocityPid.calculate(motorSim.getAngularVelocityRadPerSec()) + ffVolts;
            }
        }

        motorSim.setInputVoltage(appliedVolts);

        motorSim.update(Constants.loopPeriod);

        inputs.connected = true;
        inputs.positionRad = motorSim.getAngularPositionRad();
        inputs.velocityRadPerSec = motorSim.getAngularVelocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = Math.abs(motorSim.getCurrentDrawAmps());
    }

    @Override
    public void setPositionPIDF(LoggedTunablePIDF newGains) {
        System.out.println("Setting motor position gains");
        positionPid = newGains.toPID();
    }

    @Override
    public void setVelocityPIDF(LoggedTunablePIDF newGains) {
        System.out.println("Setting motor velocity gains");
        velocityFeedforward = newGains.toSimpleFF();
        velocityPid = newGains.toPID();
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
        System.out.println("Setting motor neutral mode to " + neutralMode);
    }

    @Override
    public void setRequest(RequestType type, double value) {
        switch (type) {
            case VoltageVolts -> {
                appliedVolts = value;
                closedLoop = false;
            }
            case PositionRad -> {
                closedLoop = true;
                positionControl = true;
                positionPid.setSetpoint(value);
            }
            case VelocityRadPerSec -> {
                closedLoop = true;
                positionControl = false;
                ffVolts = velocityFeedforward.calculate(value);
                velocityPid.setSetpoint(value);
            }
        }
    }
}
