package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.simulation.packets.ServerUpdatePacket;
import lombok.RequiredArgsConstructor;
import org.ironmaple.simulation.SimulatedArena;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class MultiplayerSimServer {
    public static final int SERVER_PORT = 12348;

    private final List<Client> clients = new ArrayList<>();
    private byte nextClientId = 1;

    private final DatagramSocket socket;
    private final byte[] buf = new byte[1024];

    public MultiplayerSimServer() {
        try {
            socket = new DatagramSocket(SERVER_PORT);
            socket.setSoTimeout(500);
        } catch (SocketException e) {
            throw new RuntimeException(e);
        }
        new Thread(this::run, "MultiplayerSimServer").start();
        Runtime.getRuntime().addShutdownHook(new Thread(() -> {
            System.out.println("Closing socket");
            try {
                socket.close();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }));
    }

    private void run() {
        SmartDashboard.putBoolean("Multiplayer/Server/Started", true);
        while (!socket.isClosed()) {
            try {
                SmartDashboard.putBoolean("Multiplayer/Server/Connected", socket.isConnected());

                // Wait for client update
                DatagramPacket packet = new DatagramPacket(buf, buf.length);
                SmartDashboard.putNumber("Multiplayer/Server/LastTryReceive", System.currentTimeMillis());
                socket.receive(packet);
                SmartDashboard.putNumber("Multiplayer/Server/LastReceive", System.currentTimeMillis());

                int port = packet.getPort();
                InetAddress address = packet.getAddress();

                // Send back server update
                dataLock.lock();
                byte[] rawPacket;
                try {
                    rawPacket = new ServerUpdatePacket(
                            new byte[]{0},
                            new Pose2d[]{hostPose},
                            fuelPoses
                    ).toBytes();
                } finally {
                    dataLock.unlock();
                }
                packet = new DatagramPacket(rawPacket, rawPacket.length, address, port);
                SmartDashboard.putNumber("Multiplayer/Server/LastTrySend", System.currentTimeMillis());
                socket.send(packet);
                SmartDashboard.putNumber("Multiplayer/Server/LastSend", System.currentTimeMillis());
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    @RequiredArgsConstructor
    private class Client {
        private final byte id;
        //private final SelfControlledSwerveDriveSimulation driveSimulation;
    }

    private static Lock dataLock = new ReentrantLock();
    private static Pose2d hostPose = new Pose2d();
    private static Translation3d[] fuelPoses = new Translation3d[0];

    public static void updateData() {
        if (dataLock.tryLock()) {
            try {
                hostPose = SimManager.get().driveSimulation.getSimulatedDriveTrainPose();
                fuelPoses = Arrays.stream(SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel")).map(Pose3d::getTranslation).toArray(Translation3d[]::new);
            } finally {
                dataLock.unlock();
            }
        }
    }
}
