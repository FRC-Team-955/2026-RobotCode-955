package frc.robot.simulation;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.simulation.packets.ClientConnectionPacket;
import frc.robot.simulation.packets.ClientUpdatePacket;
import frc.robot.simulation.packets.ServerUpdatePacket;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;

public class MultiplayerSimServer {
    public static final int SERVER_PORT = 12348;

    private static final List<Client> clients = new ArrayList<>();

    private final DatagramSocket socket;
    private final byte[] receiveBuf = new byte[ClientUpdatePacket.SIZE];

    public MultiplayerSimServer() {
        // Add server/host client
        clients.add(new Client(0, null, () -> SimManager.get().driveSimulation.getSimulatedDriveTrainPose()));

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
                DatagramPacket packet = new DatagramPacket(receiveBuf, receiveBuf.length);
                SmartDashboard.putNumber("Multiplayer/Server/LastTryReceive", System.currentTimeMillis());
                socket.receive(packet);
                SmartDashboard.putNumber("Multiplayer/Server/LastReceive", System.currentTimeMillis());

                // Process client update
                if (packet.getLength() == ClientConnectionPacket.SIZE) {
                    ClientConnectionPacket parsed = ClientConnectionPacket.fromBytes(packet.getData(), packet.getLength());
                    boolean validId = true;
                    for (Client client : clients) {
                        if (client.id == parsed.id()) {
                            validId = false;
                            break;
                        }
                    }
                    if (validId) {
                        clients.add(Client.create(parsed.id()));
                        System.out.println("Adding client " + parsed.id());
                    } else {
                        System.out.println("Invalid client " + parsed.id());
                    }
                } else if (packet.getLength() == ClientUpdatePacket.SIZE) {
                    ClientUpdatePacket parsed = ClientUpdatePacket.fromBytes(packet.getData(), packet.getLength());
                    if (parsed.id() != 0) {
                        for (Client client : clients) {
                            if (client.id == parsed.id()) {
                                client.driveSimulation.runChassisSpeeds(parsed.wantedSpeeds(), new Translation2d(), false, false);
                            }
                        }
                    }
                }

                // Send back server update
                dataLock.lock();
                byte[] rawPacket;
                try {
                    rawPacket = new ServerUpdatePacket(
                            clients.stream().mapToInt(c -> c.id).toArray(),
                            robotPoses,
                            fuelPoses
                    ).toBytes();
                } finally {
                    dataLock.unlock();
                }
                packet = new DatagramPacket(rawPacket, rawPacket.length, packet.getAddress(), packet.getPort());
                SmartDashboard.putNumber("Multiplayer/Server/LastTrySend", System.currentTimeMillis());
                socket.send(packet);
                SmartDashboard.putNumber("Multiplayer/Server/LastSend", System.currentTimeMillis());
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    private record Client(int id, SelfControlledSwerveDriveSimulation driveSimulation, Supplier<Pose2d> poseSupplier) {
        public static Client create(int id) {
            var driveSimulation = new SelfControlledSwerveDriveSimulation(new SwerveDriveSimulation(
                    SimManager.driveSimulationConfig,
                    new Pose2d(1, 1 + id, new Rotation2d())
            ));
            SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation.getDriveTrainSimulation());
            return new Client(
                    id,
                    driveSimulation,
                    driveSimulation::getActualPoseInSimulationWorld
            );
        }
    }

    private static final Lock dataLock = new ReentrantLock();
    private static Pose2d[] robotPoses = new Pose2d[0];
    private static Translation3d[] fuelPoses = new Translation3d[0];

    public static void updateData() {
        if (dataLock.tryLock()) {
            try {
                robotPoses = clients.stream().map(c -> c.poseSupplier.get()).toArray(Pose2d[]::new);
                fuelPoses = Arrays.stream(SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel")).map(Pose3d::getTranslation).toArray(Translation3d[]::new);
            } finally {
                dataLock.unlock();
            }
        }
    }
}
