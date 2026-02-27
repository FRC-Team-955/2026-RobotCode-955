package frc.lib.network;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/**
 * Units (velocity control additions are in brackets):
 * <ul>
 *     <li><code>kP</code>: volts / (rad [per sec] of error)</li>
 *     <li><code>kI</code>: volts / (rad [per sec] of accumulated error)</li>
 *     <li><code>kD</code>: volts / (rad per sec [per sec] of derivative error)</li>
 *     <ul>
 *         <li>position control derivative is velocity</li>
 *         <li>velocity control derivative is acceleration</li>
 *     </ul>
 *     <li><code>kS</code>: volts</li>
 *     <li><code>kV</code>: volts / (rad per sec)</li>
 *     <li><code>kA</code>: volts / (rad per sec per sec)</li>
 *     <li><code>kG</code>: volts</li>
 * </ul>
 */
public class LoggedTunablePIDF {
    private final String name;
    private LoggedTunableNumber kP;
    private LoggedTunableNumber kI;
    private LoggedTunableNumber kD;
    private LoggedTunableNumber kS;
    private LoggedTunableNumber kV;
    private LoggedTunableNumber kA;
    private LoggedTunableNumber kG;
    private GravityTypeValue gravityType;
    private StaticFeedforwardSignValue staticFeedforwardSign;

    public LoggedTunablePIDF(String name) {
        this.name = name;
    }

    public LoggedTunablePIDF withP(double kP) {
        this.kP = new LoggedTunableNumber(name + "/kP", kP);
        return this;
    }

    public LoggedTunablePIDF withI(double kI) {
        this.kI = new LoggedTunableNumber(name + "/kI", kI);
        return this;
    }

    public LoggedTunablePIDF withD(double kD) {
        this.kD = new LoggedTunableNumber(name + "/kD", kD);
        return this;
    }

    public LoggedTunablePIDF withS(double kS, StaticFeedforwardSignValue staticFeedforwardSign) {
        this.kS = new LoggedTunableNumber(name + "/kS", kS);
        this.staticFeedforwardSign = staticFeedforwardSign;
        return this;
    }

    public LoggedTunablePIDF withV(double kV) {
        this.kV = new LoggedTunableNumber(name + "/kV", kV);
        return this;
    }

    public LoggedTunablePIDF withA(double kA) {
        this.kA = new LoggedTunableNumber(name + "/kA", kA);
        return this;
    }

    public LoggedTunablePIDF withG(double kG, GravityTypeValue gravityType) {
        this.kG = new LoggedTunableNumber(name + "/kG", kG);
        this.gravityType = gravityType;
        return this;
    }

    public void applySpark(ClosedLoopConfig config, ClosedLoopSlot slot) {
        // We do spark unit conversions on controller so no need for unit conversions
        if (kP != null) config.p(kP.get(), slot);
        if (kI != null) config.i(kI.get(), slot);
        if (kD != null) config.d(kD.get(), slot);
        if (kS != null) config.feedForward.kS(kS.get(), slot);
        if (kV != null) config.feedForward.kV(kV.get(), slot);
        if (kA != null) config.feedForward.kA(kA.get(), slot);
        if (kG != null) {
            switch (gravityType) {
                case Elevator_Static -> config.feedForward.kG(kG.get(), slot);
                case Arm_Cosine -> {
                    config.feedForward.kCos(kG.get(), slot);
                    // Convert radians to rotations
                    // See https://docs.revrobotics.com/revlib/spark/closed-loop/feed-forward-control#kcosratio-ratio-constant-for-use-with-kcos
                    config.feedForward.kCosRatio(1.0 / (2.0 * Math.PI), slot);
                }
            }

        }
    }

    public SlotConfigs toPhoenix() {
        SlotConfigs configs = new SlotConfigs();

        // See top level javadoc for LoggedTunablePIDF units
        // Phoenix unit is rotations

        // Why multiply by 2π?
        //    volts      2 π rad
        // ----------- * -------
        // rad per sec    1 rot

        if (kP != null) {
            configs.kP = kP.get() * 2 * Math.PI;
        }

        if (kI != null) {
            configs.kI = kI.get() * 2 * Math.PI;
        }

        if (kD != null) {
            configs.kD = kD.get() * 2 * Math.PI;
        }

        if (kS != null) {
            configs.kS = kS.get(); // kS stays the same
            configs.StaticFeedforwardSign = staticFeedforwardSign;
        }

        if (kV != null) {
            configs.kV = kV.get() * 2 * Math.PI;
        }

        if (kA != null) {
            configs.kA = kA.get() * 2 * Math.PI;
        }

        if (kG != null) {
            configs.kG = kG.get(); // kG stays the same
            configs.GravityType = gravityType;
        }

        return configs;
    }

    public void applyPID(PIDController controller) {
        controller.setPID(
                this.kP != null ? this.kP.get() : 0.0,
                this.kI != null ? this.kI.get() : 0.0,
                this.kD != null ? this.kD.get() : 0.0
        );
    }

    public PIDController toPID() {
        return new PIDController(
                this.kP != null ? this.kP.get() : 0.0,
                this.kI != null ? this.kI.get() : 0.0,
                this.kD != null ? this.kD.get() : 0.0
        );
    }

    public PIDController toPID(double errorTolerance, double errorDerivativeTolerance) {
        var pid = toPID();
        pid.setTolerance(errorTolerance, errorDerivativeTolerance);
        return pid;
    }

    public PIDController toPIDWrapRadians() {
        var pid = new PIDController(
                this.kP != null ? this.kP.get() : 0.0,
                this.kI != null ? this.kI.get() : 0.0,
                this.kD != null ? this.kD.get() : 0.0
        );
        pid.enableContinuousInput(-Math.PI, Math.PI);
        return pid;
    }

    public PIDController toPIDWrapRadians(double errorTolerance, double errorDerivativeTolerance) {
        var pid = toPIDWrapRadians();
        pid.setTolerance(errorTolerance, errorDerivativeTolerance);
        return pid;
    }

    public SimpleMotorFeedforward toSimpleFF() {
        return new SimpleMotorFeedforward(
                this.kS != null ? this.kS.get() : 0.0,
                this.kV != null ? this.kV.get() : 0.0,
                this.kA != null ? this.kA.get() : 0.0
        );
    }

    public ArmFeedforward toArmFF() {
        return new ArmFeedforward(
                this.kS != null ? this.kS.get() : 0.0,
                this.kG != null ? this.kG.get() : 0.0,
                this.kV != null ? this.kV.get() : 0.0,
                this.kA != null ? this.kA.get() : 0.0
        );
    }

    public ElevatorFeedforward toElevatorFF() {
        return new ElevatorFeedforward(
                this.kS != null ? this.kS.get() : 0.0,
                this.kG != null ? this.kG.get() : 0.0,
                this.kV != null ? this.kV.get() : 0.0,
                this.kA != null ? this.kA.get() : 0.0
        );
    }

    public boolean hasChanged() {
        if (kP != null && kP.hasChanged()) return true;
        if (kI != null && kI.hasChanged()) return true;
        if (kD != null && kD.hasChanged()) return true;
        if (kS != null && kS.hasChanged()) return true;
        if (kV != null && kV.hasChanged()) return true;
        if (kA != null && kA.hasChanged()) return true;
        return kG != null && kG.hasChanged();
    }
}
