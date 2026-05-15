package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructSubscriber;
import frc.robot.Constants;
import frc.robot.simulation.packets.ClientConnectionPacket;
import frc.robot.simulation.packets.ClientUpdatePacket;
import frc.robot.simulation.packets.ServerUpdatePacket;
import org.jetbrains.annotations.Nullable;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class MultiplayerSimClient {
    public static final int CLIENT_PORT = 12349;

    private final DatagramSocket socket;
    private final InetAddress address;
    private byte[] sendBuf = new byte[ClientUpdatePacket.SIZE];
    private final byte[] receiveBuf = new byte[ServerUpdatePacket.MAX_SIZE];

    private int id = 0;

    private final StructSubscriber<ChassisSpeeds> setpointSpeedsSubscriber = NetworkTableInstance.getDefault()
            .getStructTopic("/AdvantageKit/RealOutputs/Drive/ChassisSpeeds/SetpointOptimized", ChassisSpeeds.struct)
            .subscribe(new ChassisSpeeds(), PubSubOption.periodic(Constants.loopPeriod));

    public MultiplayerSimClient() {
        try {
            socket = new DatagramSocket(CLIENT_PORT);
            socket.setSoTimeout(500);
            address = InetAddress.getLoopbackAddress();
        } catch (SocketException e) {
            throw new RuntimeException(e);
        }
        new Thread(this::run, "MultiplayerSimClient").start();
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
        while (!socket.isClosed()) {
            try {
                MultiplayerLogger.logStatus("Client " + (socket.isConnected() ? "connected" : "disconnected"));

                // Send client update, which serves as a request for a server update
                DatagramPacket packet = new DatagramPacket(sendBuf, sendBuf.length, address, MultiplayerSimServer.SERVER_PORT);
                MultiplayerLogger.logTrySend();
                socket.send(packet);
                MultiplayerLogger.logSend();

                // Receive server update
                packet = new DatagramPacket(receiveBuf, receiveBuf.length);
                MultiplayerLogger.logTryReceive();
                socket.receive(packet);
                MultiplayerLogger.logReceive();

                // Process server update
                var update = ServerUpdatePacket.fromBytes(packet.getData(), packet.getLength());
                MultiplayerLogger.logUpdate(update);

                // Respond and prepare next client update
                // Check if our ID is connected
                boolean needsToConnect = true;
                for (int connectedId : update.connectedIds()) {
                    if (connectedId == id) {
                        needsToConnect = false;
                        break;
                    }
                }
                if (id == 0 || needsToConnect) {
                    // Need to connect
                    for (int connectedId : update.connectedIds()) {
                        if (connectedId >= id) {
                            id = connectedId + 1;
                        }
                    }
                    sendBuf = new ClientConnectionPacket(id).toBytes();
                } else {
                    // Connected, send update packets and update our local simulation
                    sendBuf = new ClientUpdatePacket(
                            id,
                            setpointSpeedsSubscriber.get(),
                            false,
                            0.0,
                            0.0
                    ).toBytes();

                    for (int i = 0; i < update.connectedIds().length; i++) {
                        if (update.connectedIds()[i] == id) {
                            if (update.robotPoses().length > i) {
                                dataLock.lock();
                                try {
                                    poseFromLastUpdate = update.robotPoses()[i];
                                } finally {
                                    dataLock.unlock();
                                }
                            }
                        }
                    }
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        //socket.close();
    }

    private static final Lock dataLock = new ReentrantLock();
    private static Pose2d poseFromLastUpdate = null;

    public static @Nullable Pose2d getServerPose() {
        dataLock.lock();
        try {
            if (poseFromLastUpdate != null) {
                Pose2d toReturn = poseFromLastUpdate;
                poseFromLastUpdate = null;
                return toReturn;
            }
            return null;
        } finally {
            dataLock.unlock();
        }
    }
}
