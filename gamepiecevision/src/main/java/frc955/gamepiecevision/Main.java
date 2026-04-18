// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc955.gamepiecevision;


import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import org.photonvision.jni.LibraryLoader;
import org.photonvision.jni.TimeSyncServer;
import org.photonvision.timesync.TimeSyncSingleton;

public final class Main {
    private Main() {
    }

    public static void main(String... args) {
        // Load natives
        System.out.println("Loading natives");
        LibraryLoader.loadWpiLibraries();
        LibraryLoader.loadTargeting();

        // Setup NetworkTables
        System.out.println("Setting up NT");
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startClient4("frc955.gamepiecevision");
        inst.setServerTeam(955);
        if (System.getenv("GPVLOCAL") != null) {
            inst.setServer("127.0.0.1");
        }

        // Manually create TimeSyncSingleton instance to avoid attempt to load photontargetingJNI
        System.out.println("Fixing TimeSyncSingleton");
        try {
            var instance = TimeSyncSingleton.class.getDeclaredField("INSTANCE");
            instance.setAccessible(true);
            var server = new TimeSyncServer(5810);
            server.start();
            instance.set(null, server);
        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new RuntimeException(e);
        }

        // Start code
        System.out.println("Starting robot");
        RobotBase.startRobot(Robot::new);
    }
}
