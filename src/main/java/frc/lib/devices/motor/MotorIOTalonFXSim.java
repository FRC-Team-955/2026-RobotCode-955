package frc.lib.devices.motor;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.SimManager;

public class MotorIOTalonFXSim extends MotorIOTalonFX {
    private final TalonFXSimState talonSim;
    private final MechanismSim mechanismSim;

    public MotorIOTalonFXSim(TalonFXConfiguration config, double initialPositionRad, MechanismSim.Builder mechanismSimBuilder) {
        // We tell the TalonFX that it has an initial position of 0 because setRawRotorPosition
        // will take into account initial position. If we give it the real initial position, the
        // position of the motor will double and bad things will happen.
        super(SimManager.getNewCANId(), config, 0.0);

        talonSim = talon.getSimState();
        talonSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);

        // A single kraken is usually not correct, but it's good enough for sim
        mechanismSim = mechanismSimBuilder.build(DCMotor.getKrakenX60(1), initialPositionRad, config.Feedback.SensorToMechanismRatio);
    }

    @Override
    public void updateInputs(MotorIOInputsAutoLogged inputs) {
        mechanismSim.update(talonSim);

        super.updateInputs(inputs);
    }

    @Override
    public void setEncoderPosition(double positionRad) {
        mechanismSim.setMechanismPositionRad.accept(positionRad);

        // Skip setting the TalonFX encoder position for the same reasons as why we
        // tell the TalonFX that it has an initial position of 0 in the constructor.
    }
}
