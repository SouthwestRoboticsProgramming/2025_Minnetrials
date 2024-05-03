package com.swrobotics.robot.subsystems.tagtracker;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Function;

public final class TagTrackerCamera {
    public static final class PoseEstimate {
        public final double error;
        public final Pose3d pose;

        public PoseEstimate(double[] data, int i) {
            error = data[i];

            double tx = data[i + 1];
            double ty = data[i + 2];
            double tz = data[i + 3];
            Translation3d translation = new Translation3d(tx, ty, tz);

            double qw = data[i + 4];
            double qx = data[i + 5];
            double qy = data[i + 6];
            double qz = data[i + 7];
            Rotation3d rotation = new Rotation3d(new Quaternion(qw, qx, qy, qz));

            pose = new Pose3d(translation, rotation);
        }

        @Override
        public String toString() {
            return "PoseEstimate{" +
                    "error=" + error +
                    ", pose=" + pose +
                    '}';
        }
    }

    public static final class EstimateInput {
        public final double timestamp;
        public final PoseEstimate poseA;
        public final PoseEstimate poseB;
        public final int[] visibleTagIds;

        public static EstimateInput decode(long ntTimestamp, double[] data) {
            double timestamp = ntTimestamp / 1_000_000.0;

            int count = (int) data[0];
            if (count == 0)
                return null;

            PoseEstimate poseA = new PoseEstimate(data, 1);
            PoseEstimate poseB = count == 2 ? new PoseEstimate(data, 9) : null;

            int tagsOffset = count == 2 ? 17 : 9;
            int[] visibleTagIds = new int[data.length - tagsOffset - 1];
            for (int i = tagsOffset; i < data.length - 1; i++) {
                visibleTagIds[i - tagsOffset] = (int) data[i];
            }

            double timeOffset = data[data.length - 1];
            return new EstimateInput(timestamp - timeOffset, poseA, poseB, visibleTagIds);
        }

        private EstimateInput(double timestamp, PoseEstimate poseA, PoseEstimate poseB, int[] visibleTagIds) {
            this.timestamp = timestamp;
            this.poseA = poseA;
            this.poseB = poseB;
            this.visibleTagIds = visibleTagIds;
        }

        @Override
        public String toString() {
            return "EstimateInput{" +
                    "timestamp=" + timestamp +
                    ", poseA=" + poseA +
                    ", poseB=" + poseB +
                    ", visibleTagIds=" + Arrays.toString(visibleTagIds) +
                    '}';
        }
    }

    private final String name;
//    private final Transform3d toRobotTransform;
    private final Function<Pose3d, Pose3d> cameraToRobot;
    private final double maxTrustDistance;

    private final DoubleArraySubscriber posesSub;

    private final BooleanPublisher autoExposurePub;
    private final DoublePublisher exposurePub;
    private final DoublePublisher gainPub;
    private final DoublePublisher targetFpsPub;

    public TagTrackerCamera(String name, NetworkTable table, Function<Pose3d, Pose3d> cameraToRobot, CameraCaptureProperties props, double maxTrustDistance) {
        this.name = name;
//        this.toRobotTransform = toRobotTransform;
        this.cameraToRobot = cameraToRobot;
        this.maxTrustDistance = maxTrustDistance;

        posesSub = table.getSubTable("Outputs").getDoubleArrayTopic("poses").subscribe(
                new double[]{0},
                PubSubOption.sendAll(true),
                PubSubOption.keepDuplicates(true));

        NetworkTable configTable = table.getSubTable("Config");
        autoExposurePub = configTable.getBooleanTopic("Auto Exposure").publish();
        exposurePub = configTable.getDoubleTopic("Exposure").publish();
        gainPub = configTable.getDoubleTopic("Gain").publish();
        targetFpsPub = configTable.getDoubleTopic("Target FPS").publish();

        autoExposurePub.set(props.isAutoExposure());
        exposurePub.set(props.getExposure());
        gainPub.set(props.getGain());
        targetFpsPub.set(props.getTargetFps());
    }

    public Function<Pose3d, Pose3d> getToRobotTransform() {
        return cameraToRobot;
    }

    public double getMaxTrustDistance() {
        return maxTrustDistance;
    }

    public List<EstimateInput> getEstimates() {
        TimestampedDoubleArray[] queuedValues = posesSub.readQueue();

        List<EstimateInput> estimates = new ArrayList<>();
        for (TimestampedDoubleArray value : queuedValues) {
            EstimateInput input = EstimateInput.decode(value.timestamp, value.value);
            if (input != null)
                estimates.add(input);
        }

        return estimates;
    }
}
