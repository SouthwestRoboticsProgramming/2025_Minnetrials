package com.swrobotics.robot.pathfinding;

import edu.wpi.first.wpilibj.RobotBase;

import java.nio.file.Path;
import java.nio.file.Paths;

public final class PathfindingJNI {
    // Use debug for testing, release for performance
//    private static final String PROFILE = "debug";
    private static final String PROFILE = "release";

    private static final String LIBRARY_NAME = "libpathfinding_jni";

    static {
        RuntimeType type = RuntimeType.getCurrent();
        Path p = type.getLibraryPath();
        System.out.println("Pathfinding JNI path is " + p.toAbsolutePath());

        try {
            System.load(p.toAbsolutePath().toString());
        } catch (UnsatisfiedLinkError e) {
            if (type == RuntimeType.SIMULATION_WINDOWS || type == RuntimeType.SIMULATION_LINUX) {
                // Give a little reminder
                System.err.println("Failed to load Pathfinding JNI library. Make sure the library is built using '"
                    + (PROFILE.equals("release") ? "cargo build --release" : "cargo build")
                    + "'.");
            }
            throw e;
        }
    }

    // These functions are implemented in src/main/rust/lib.rs

    public static native long newObstacleList(); // Returns obstacle list handle
    public static native void addCircle(long obsHandle, double posX, double posY, double radius);
    public static native void addPolygon(long obsHandle, double[] vertices);

    // Takes ownership of the obstacle list - don't use it afterward!
    public static native long buildEnvironment(long obsHandle, double avoidanceRadius); // Returns environment handle

    // Returns null if path not found
    public static native double[] findPath(long envHandle, double startX, double startY, double goalX, double goalY);


    // Not perfect platform compatibility, but good enough for our uses
    private enum RuntimeType {
        ROBORIO {
            @Override
            Path getLibraryPath() {
                // FIXME: Don't do this
                //  Still need to figure out how to compile Rust for RoboRIO
                throw new RuntimeException("TODO");
            }
        },
        SIMULATION_WINDOWS {
            @Override
            Path getLibraryPath() {
                return Paths.get("target\\" + PROFILE + "\\" + LIBRARY_NAME + ".dll");
            }
        },
        SIMULATION_LINUX {
            @Override
            Path getLibraryPath() {
                return Paths.get("target/" + PROFILE + "/" + LIBRARY_NAME + ".so");
            }
        };

        abstract Path getLibraryPath();

        static RuntimeType getCurrent() {
            if (RobotBase.isReal())
                return ROBORIO;
            String os = System.getProperty("os.name");
            if (os == null || os.toLowerCase().startsWith("windows"))
                return SIMULATION_WINDOWS;
            else
                return SIMULATION_LINUX;
        }
    }
}
