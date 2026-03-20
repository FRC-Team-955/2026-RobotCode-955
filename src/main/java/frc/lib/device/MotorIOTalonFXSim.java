package frc.lib.device;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;

public class MotorIOTalonFXSim extends MotorIOTalonFX {
    private final TalonFXSimState talonSim;
    private final MechanismSim mechanismSim;

    public MotorIOTalonFXSim(int canID, TalonFXConfiguration config, double initialPositionRad, MechanismSim.Builder mechanismSimBuilder) {
        super(canID, config, initialPositionRad);

        talonSim = talon.getSimState();
        talonSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);

        // A single kraken is usually not correct, but it's good enough for sim
        mechanismSim = mechanismSimBuilder.build(DCMotor.getKrakenX60(1), initialPositionRad);
    }

    @Override
    public void updateInputs(MotorIOInputsAutoLogged inputs) {
        mechanismSim.update(talonSim);

        super.updateInputs(inputs);
    }

    @Override
    public void setEncoderPosition(double positionRad) {
        super.setEncoderPosition(positionRad);
    }
}
