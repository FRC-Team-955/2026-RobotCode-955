package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.LinearSystem;

public class GamePieceTracker {

    public KalmanFilter<N4, N2, N2> filter;

    public GamePieceTracker() {
        double dt = 0.02;
        //        Matrix<N4, N4> A = new Matrix<>(Nat.N4(), Nat.N4());
        Matrix<N4, N4> A = MatBuilder.fill(Nat.N4(), Nat.N4(), 1, 0, dt, 0,
                0, 1, 0, dt,
                0, 0, 1, 0,
                0, 0, 0, 1);

        Matrix<N4, N2> B = MatBuilder.fill(Nat.N4(), Nat.N2(), 0, 0,
                0, 0,
                1, 0,
                0, 1);
        //Matrix<N4, N2> B = new Matrix<>(Nat.N4(), Nat.N2());

        Matrix<N2, N4> C = MatBuilder.fill(Nat.N2(), Nat.N4(), 1, 0, 0, 0,
                0, 1, 0, 0);
        Matrix<N2, N2> D = MatBuilder.fill(Nat.N2(), Nat.N2(), 0, 0, 0, 0);

        var plant = new LinearSystem<>(
                A,
                B,
                C,
                D
        );

        filter = new KalmanFilter<>(
                Nat.N4(), Nat.N2(),
                plant,
                VecBuilder.fill(0.02, 0.02, 1.0, 1.0),
                VecBuilder.fill(0.3, 0.3),
                dt
        );
    }
}
