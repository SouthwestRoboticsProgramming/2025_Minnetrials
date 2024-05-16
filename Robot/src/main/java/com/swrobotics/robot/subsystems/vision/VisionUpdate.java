package com.swrobotics.robot.subsystems.vision;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;

public final class VisionUpdate {
    public final double timestamp;
    public final Pose2d estPose;
    public final Vector<N3> stdDevs;

    public VisionUpdate(double timestamp, Pose2d estPose, Vector<N3> stdDevs) {
        this.timestamp = timestamp;
        this.estPose = estPose;
        this.stdDevs = stdDevs;
    }

    @Override
    public String toString() {
        return "VisionUpdate{" +
                "timestamp=" + timestamp +
                ", estPose=" + estPose +
                ", stdDevs=" + stdDevs +
                '}';
    }
}
