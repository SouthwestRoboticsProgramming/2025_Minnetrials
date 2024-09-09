package com.swrobotics.robot.subsystems.vision;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;

/**
 * One vision estimate.
 */
public final class VisionUpdate {
    /**
     * The timestamp at which the camera frame was captured. This is aligned to
     * {@code Timer.getFPGATimestamp()}.
     */
    public final double timestamp;

    /** The estimated 2D pose of the robot */
    public final Pose2d estPose;

    /**
     * The standard deviation of the estimate, as (x, y, theta). Updates with
     * lower standard deviation are trusted more than updates with higher SD.
     */
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
