package com.swrobotics.robot.subsystems.vision.tagtracker;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

import java.io.ByteArrayInputStream;
import java.io.DataInput;
import java.io.DataInputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Function;

public final class TagTrackerCamera {
    public static final class PoseEstimate {
        public final float error;
        public final Pose3d pose;

        public PoseEstimate(DataInput in) throws IOException {
            error = in.readFloat();

            double tx = in.readDouble();
            double ty = in.readDouble();
            double tz = in.readDouble();
            Translation3d translation = new Translation3d(tx, ty, tz);

            double qw = in.readDouble();
            double qx = in.readDouble();
            double qy = in.readDouble();
            double qz = in.readDouble();
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
        public final Translation2d[][] visibleTagCorners;

        public static EstimateInput decode(long ntTimestamp, byte[] rawData) {
            DataInput in = new DataInputStream(new ByteArrayInputStream(rawData));
            try {
                double timestamp = ntTimestamp / 1_000_000.0;

                boolean hasEstimate = in.readBoolean();
                if (!hasEstimate)
                    return null;

                PoseEstimate poseA = new PoseEstimate(in);
                PoseEstimate poseB = null;
                int tagCount = 1;
                if (in.readBoolean()) {
                    poseB = new PoseEstimate(in);
                    tagCount = in.readUnsignedByte();
                }

                int[] visibleTagIds = new int[tagCount];
                Translation2d[][] visibleTagCorners = new Translation2d[tagCount][4];
                for (int i = 0; i < tagCount; i++) {
                    visibleTagIds[i] = in.readUnsignedByte();

                    Translation2d[] cornerSet = visibleTagCorners[i];
                    for (int j = 0; j < 4; j++) {
                        double x = in.readUnsignedShort();
                        double y = in.readUnsignedShort();
                        cornerSet[j] = new Translation2d(x, y);
                    }
                }

                double timeOffset = in.readDouble();
                return new EstimateInput(
                        timestamp - timeOffset,
                        poseA, poseB,
                        visibleTagIds,
                        visibleTagCorners
                );
            } catch (IndexOutOfBoundsException e) {
                DriverStation.reportWarning("TagTracker did not provide enough data!", false);
            } catch (IOException impossible) {
                DriverStation.reportWarning("TagTracker: Somehow an IOException appeared", false);
                impossible.printStackTrace();
            }
            return null;
        }

        private EstimateInput(double timestamp, PoseEstimate poseA, PoseEstimate poseB, int[] visibleTagIds, Translation2d[][] visibleTagCorners) {
            this.timestamp = timestamp;
            this.poseA = poseA;
            this.poseB = poseB;
            this.visibleTagIds = visibleTagIds;
            this.visibleTagCorners = visibleTagCorners;
        }

        @Override
        public String toString() {
            return "EstimateInput{" +
                    "timestamp=" + timestamp +
                    ", poseA=" + poseA +
                    ", poseB=" + poseB +
                    ", visibleTagIds=" + Arrays.toString(visibleTagIds) +
                    ", visibleTagCorners=" + Arrays.deepToString(visibleTagCorners) +
                    '}';
        }
    }

    private final String name;
//    private final Transform3d toRobotTransform;
    private final Function<Pose3d, Pose3d> cameraToRobot;
    private final double maxTrustDistance;

    private final TagTrackerCameraIO io;
    private final TagTrackerCameraIO.Inputs inputs;

    public TagTrackerCamera(String name, NetworkTable table, Function<Pose3d, Pose3d> cameraToRobot, TagTrackerCaptureProperties captureProps, double maxTrustDistance) {
        this.name = name;
//        this.toRobotTransform = toRobotTransform;
        this.cameraToRobot = cameraToRobot;
        this.maxTrustDistance = maxTrustDistance;

        io = new NTCameraIO(table);
        inputs = new TagTrackerCameraIO.Inputs();

        io.setCaptureProperties(captureProps);
    }

    public Function<Pose3d, Pose3d> getToRobotTransform() {
        return cameraToRobot;
    }

    public double getMaxTrustDistance() {
        return maxTrustDistance;
    }

    public List<EstimateInput> getEstimates() {
        io.updateInputs(inputs);
        Logger.processInputs("TagTracker/Camera/" + name, inputs);

        List<EstimateInput> estimates = new ArrayList<>();

        boolean hadFrame = false;
        for (int i = 0; i < inputs.timestamps.length; i++) {
            long timestamp = inputs.timestamps[i];
            byte[] data = inputs.framePackedData[i];

            EstimateInput input = EstimateInput.decode(timestamp, data);
            if (input != null)
                estimates.add(input);
            hadFrame = true;
        }

        if (hadFrame) {
            List<Translation2d> corners = new ArrayList<>();
            if (!estimates.isEmpty()) {
                EstimateInput latest = estimates.get(estimates.size() - 1);
                for (Translation2d[] cornerSet : latest.visibleTagCorners) {
                    corners.addAll(Arrays.asList(cornerSet));
                }
            }
            Logger.recordOutput("TagTracker/Camera/" + name + "/Tag Corners", corners.toArray(new Translation2d[0]));
        }

        return estimates;
    }
}
