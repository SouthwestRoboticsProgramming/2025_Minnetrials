package com.swrobotics.robot.subsystems.tagtracker;

import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.utils.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

public final class TagTrackerInput {
    public static final class CameraInfo {
        public final String name;
        public final Function<Pose3d, Pose3d> cameraToRobot;
        public final CameraCaptureProperties captureProps;
        public final double maxTrustDistance;
//        public final Pose3d robotRelPose;

        public CameraInfo(String name, Function<Pose3d, Pose3d> cameraToRobot, CameraCaptureProperties captureProps, double maxTrustDistance) {
            this.name = name;
            this.cameraToRobot = cameraToRobot;
//            this.robotRelPose = robotRelPose;
            this.captureProps = captureProps;
            this.maxTrustDistance = maxTrustDistance;
        }
    }

    public static final class VisionUpdate {
        public final double timestamp;
        public final Pose2d estPose;
        public final Vector<N3> stdDevs;

        public VisionUpdate(double timestamp, Pose2d estPose, Vector<N3> stdDevs) {
            this.timestamp = timestamp;
            this.estPose = estPose;
            this.stdDevs = stdDevs;
        }
    }

    private final TagTrackerEnvironment environment;
    private final TagTrackerCamera[] cameras;

    public TagTrackerInput(CameraInfo... infos) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("TagTracker");
        environment = new TagTrackerEnvironment(table.getDoubleArrayTopic("environment"));

        NetworkTable camerasTable = table.getSubTable("Cameras");
        cameras = new TagTrackerCamera[infos.length];
        for (int i = 0; i < infos.length; i++) {
            CameraInfo info = infos[i];
            cameras[i] = new TagTrackerCamera(
                    info.name,
                    camerasTable.getSubTable(info.name),
                    info.cameraToRobot,
                    info.captureProps,
                    info.maxTrustDistance);
        }
    }

    private boolean outOfRange(double val, double min, double max) {
        return val < min || val > max;
    }

    public List<VisionUpdate> getNewUpdates() {
        environment.update();

        List<VisionUpdate> updates = new ArrayList<>();
        for (TagTrackerCamera camera : cameras) {
            for (TagTrackerCamera.EstimateInput input : camera.getEstimates()) {
                Pose3d cameraPose = null;
                Pose3d robotPose3d = null;
                if (input.poseB == null) {
                    // Only one pose available, camera sees multiple tags
                    cameraPose = input.poseA.pose;
                    robotPose3d = camera.getToRobotTransform().apply(cameraPose);
//                    robotPose3d = cameraPose.transformBy(camera.getToRobotTransform());
                } else {
                    // Two poses available (one tag), choose the better one.
                    // Pose is only chosen if it's significantly better than the
                    // other. This ignores tags where the two errors are close to
                    // each other (i.e. facing the tag straight on)
                    double errA = input.poseA.error;
                    double errB = input.poseB.error;
                    if (errA < errB * Constants.kVisionAmbiguityThreshold)
                        cameraPose = input.poseA.pose;
                    else if (errB < errA * Constants.kVisionAmbiguityThreshold)
                        cameraPose = input.poseB.pose;

                    if (cameraPose != null)
                        robotPose3d = camera.getToRobotTransform().apply(cameraPose);
//                        robotPose3d = cameraPose.transformBy(camera.getToRobotTransform());
                }

                // Skip frame if no pose was good
                if (cameraPose == null)
                    continue;

                // Skip frame if outside field
                double borderMargin = Constants.kVisionFieldBorderMargin;
                if (outOfRange(robotPose3d.getX(), -borderMargin, Constants.kField.getWidth() + borderMargin)
                    || outOfRange(robotPose3d.getY(), -borderMargin, Constants.kField.getHeight() + borderMargin)
                    || outOfRange(robotPose3d.getZ(), -Constants.kVisionZMargin, Constants.kVisionZMargin)) {
                    continue;
                }

                Pose2d robotPose = robotPose3d.toPose2d();

                // Calculate the average distance to all the tags that are visible
                double totalTagDist = 0;
                int tagCount = 0;
                for (int tagId : input.visibleTagIds) {
                    Pose3d tagPose = environment.getPose(tagId);
                    if (tagPose != null) {
                        totalTagDist += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
                        tagCount++;
                    }
                }
                double avgTagDist = totalTagDist / tagCount;

                // Ignore if too far away for camera to be good
                if (avgTagDist > camera.getMaxTrustDistance())
                    continue;

                // Trust farther away tags less and frames with more tags more
                double xyStdDev = Constants.kVisionXYStdDevCoeff * MathUtil.square(avgTagDist) / tagCount;
                double thetaStdDev = Constants.kVisionThetaStdDevCoeff * MathUtil.square(avgTagDist) / tagCount;
                updates.add(new VisionUpdate(
                        input.timestamp, robotPose, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
            }
        }

        return updates;
    }

    public TagTrackerEnvironment getEnvironment() {
        return environment;
    }
}
