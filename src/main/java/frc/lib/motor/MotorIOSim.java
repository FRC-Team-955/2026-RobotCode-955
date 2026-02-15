package frc.lib.motor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.PIDF;
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

    // If using SysID values, kA and kV are the gains returned from SysID, in volts/(rad/sec) or volts/(rad/sec^2)
    public MotorIOSim(DCMotor motor, double kV, double kA, PIDF positionGains, PIDF velocityGains) {
        motorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(kV, kA),
                motor,
                0.004
        );

        positionPid = positionGains.toPID();
        velocityFeedforward = velocityGains.toSimpleFF();
        velocityPid = velocityGains.toPID();
    }

    // If using physical values, JKgMetersSquared is the moment of inertia J of the flywheel
    public MotorIOSim(double gearRatio, double JKgMetersSquared, DCMotor motor, PIDF positionGains, PIDF velocityGains) {
        motorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(motor, JKgMetersSquared, gearRatio),
                motor,
                0.004,
                0.0
        );

        velocityFeedforward = velocityGains.toSimpleFF();
        positionPid = positionGains.toPID();
        velocityPid = velocityGains.toPID();
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
    public void setPositionPIDF(PIDF newGains) {
        System.out.println("Setting motor position gains");
        positionPid = newGains.toPID();
    }

    @Override
    public void setVelocityPIDF(PIDF newGains) {
        System.out.println("Setting motor velocity gains");
        velocityFeedforward = newGains.toSimpleFF();
        velocityPid = newGains.toPID();
    }

    @Override
    public void setBrakeMode(boolean enable) {
        System.out.println("Setting motor brake mode to " + enable);
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
