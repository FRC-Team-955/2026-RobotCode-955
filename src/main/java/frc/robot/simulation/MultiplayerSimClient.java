package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.simulation.packets.ClientConnectionPacket;
import frc.robot.simulation.packets.ClientUpdatePacket;
import frc.robot.simulation.packets.ServerUpdatePacket;
import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.Logger;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class MultiplayerSimClient {
    public static final int CLIENT_PORT = 12349;

    private final DatagramSocket socket;
    private final InetAddress address;
    private byte[] sendBuf = new byte[ClientUpdatePacket.SIZE];
    private final byte[] receiveBuf = new byte[ServerUpdatePacket.MAX_SIZE];

    private final Field2d field2d = new Field2d();
    private int id = 0;
    private final StructSubscriber<ChassisSpeeds> setpointSpeedsSubscriber = NetworkTableInstance.getDefault()
            .getStructTopic("/AdvantageKit/RealOutputs/Drive/ChassisSpeeds/SetpointOptimized", ChassisSpeeds.struct)
            .subscribe(new ChassisSpeeds());

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
        SmartDashboard.putBoolean("Multiplayer/Client/Started", true);
        while (!socket.isClosed()) {
            try {
                SmartDashboard.putBoolean("Multiplayer/Client/Connected", socket.isConnected());

                // Send client update, which serves as a request for a server update
                DatagramPacket packet = new DatagramPacket(sendBuf, sendBuf.length, address, MultiplayerSimServer.SERVER_PORT);
                SmartDashboard.putNumber("Multiplayer/Client/LastTrySend", System.currentTimeMillis());
                socket.send(packet);
                SmartDashboard.putNumber("Multiplayer/Client/LastSend", System.currentTimeMillis());

                // Receive server update
                packet = new DatagramPacket(receiveBuf, receiveBuf.length);
                SmartDashboard.putNumber("Multiplayer/Client/LastTryReceive", System.currentTimeMillis());
                socket.receive(packet);
                SmartDashboard.putNumber("Multiplayer/Client/LastReceive", System.currentTimeMillis());

                // Process server update
                var update = ServerUpdatePacket.fromBytes(packet.getData(), packet.getLength());
                Logger.recordOutput("Multiplayer/Client/ConnectedIDs", update.connectedIds());
                Logger.recordOutput("Multiplayer/Client/RobotPoses", update.robotPoses());
                Logger.recordOutput("Multiplayer/Client/FuelTranslations", update.fuelTranslations());
                if (update.robotPoses().length > 0) {
                    field2d.setRobotPose(update.robotPoses()[0]);
                    for (int i = 1; i < update.robotPoses().length; i++) {
                        field2d.getObject("Robot" + i).setPose(update.robotPoses()[i]);
                    }
                }
                field2d.getObject("Fuel").setPoses(Arrays.stream(update.fuelTranslations())
                        .map(t -> new Pose2d(t.toTranslation2d(), new Rotation2d()))
                        .toList()
                );
                SmartDashboard.putData("Multiplayer/Client/Field", field2d);

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
