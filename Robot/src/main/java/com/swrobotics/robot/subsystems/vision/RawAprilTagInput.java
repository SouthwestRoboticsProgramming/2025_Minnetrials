package com.swrobotics.robot.subsystems.vision;

import com.swrobotics.lib.utils.MathUtil;
import com.swrobotics.lib.utils.Transformation3d;
import com.swrobotics.robot.config.Constants;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public abstract class RawAprilTagInput implements VisionInput {
    private final AprilTagEnvironment environment;
    private final FilterParameters filterParams;
    private final Transformation3d cameraToRobotTransform;

    public RawAprilTagInput(AprilTagEnvironment environment, FilterParameters filterParams, Transformation3d cameraToRobotTransform) {
        this.environment = environment;
        this.filterParams = filterParams;
        this.cameraToRobotTransform = cameraToRobotTransform;
    }

    protected abstract List<EstimateInput> getNewEstimates();

    private boolean outOfRange(double val, double min, double max) {
        return val < min || val > max;
    }

    @Override
    public List<VisionUpdate> getNewUpdates() {
        List<VisionUpdate> updates = new ArrayList<>();

        for (EstimateInput input : getNewEstimates()) {
            Pose3d cameraPose = null;
            if (input.poseB == null) {
                // Only one pose available
                cameraPose = input.poseA.cameraPose;
            } else {
                // Two poses available (one tag), choose the better one.
                // Pose is only chosen if it's significantly better than the
                // other. This ignores tags where the two errors are close to
                // each other (i.e. facing the tag straight on)
                double errA = input.poseA.error;
                double errB = input.poseB.error;
                if (errA < errB * filterParams.ambiguityThreshold)
                    cameraPose = input.poseA.cameraPose;
                else if (errB < errA * filterParams.ambiguityThreshold)
                    cameraPose = input.poseB.cameraPose;
            }

            // Skip frame if no pose was good
            if (cameraPose == null)
                continue;

            Pose3d robotPose3d = cameraToRobotTransform.transform(cameraPose);

            // Skip frame if outside field
            double borderMargin = filterParams.fieldBorderMargin;
            if (outOfRange(robotPose3d.getX(), -borderMargin, Constants.kField.getWidth() + borderMargin)
                    || outOfRange(robotPose3d.getY(), -borderMargin, Constants.kField.getHeight() + borderMargin)
                    || outOfRange(robotPose3d.getZ(), -filterParams.zMargin, filterParams.zMargin)) {
                continue;
            }

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
            if (avgTagDist > filterParams.maxTrustDistance)
                continue;

            // Trust farther away tags less and frames with more tags more
            double xyStdDev = filterParams.xyStdDevCoefficient * MathUtil.square(avgTagDist) / tagCount;
            double thetaStdDev = filterParams.thetaStdDevCoefficient * MathUtil.square(avgTagDist) / tagCount;
            Pose2d robotPose = robotPose3d.toPose2d();
            updates.add(new VisionUpdate(
                    input.timestamp, robotPose, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
        }

        return updates;
    }

    public static final class FilterParameters {
        public double ambiguityThreshold = 0.9;
        public double xyStdDevCoefficient = 0.01;
        public double thetaStdDevCoefficient = 0.01;
        public double maxTrustDistance = Double.POSITIVE_INFINITY; // Meters
        public double fieldBorderMargin = 0.5;
        public double zMargin = 0.75;

        public FilterParameters() {
        }

        // Copy constructor
        public FilterParameters(FilterParameters o) {
            ambiguityThreshold = o.ambiguityThreshold;
            xyStdDevCoefficient = o.xyStdDevCoefficient;
            thetaStdDevCoefficient = o.thetaStdDevCoefficient;
            maxTrustDistance = o.maxTrustDistance;
            fieldBorderMargin = o.fieldBorderMargin;
            zMargin = o.zMargin;
        }

        public FilterParameters setAmbiguityThreshold(double ambiguityThreshold) {
            this.ambiguityThreshold = ambiguityThreshold;
            return this;
        }

        public FilterParameters setXYStdDevCoefficient(double xyStdDevCoefficient) {
            this.xyStdDevCoefficient = xyStdDevCoefficient;
            return this;
        }

        public FilterParameters setThetaStdDevCoefficient(double thetaStdDevCoefficient) {
            this.thetaStdDevCoefficient = thetaStdDevCoefficient;
            return this;
        }

        public FilterParameters setMaxTrustDistance(double maxTrustDistance) {
            this.maxTrustDistance = maxTrustDistance;
            return this;
        }

        public FilterParameters setFieldBorderMargin(double fieldBorderMargin) {
            this.fieldBorderMargin = fieldBorderMargin;
            return this;
        }

        public FilterParameters setZMargin(double zMargin) {
            this.zMargin = zMargin;
            return this;
        }

        @Override
        public String toString() {
            return "FilterParameters{" +
                    "ambiguityThreshold=" + ambiguityThreshold +
                    ", xyStdDevCoefficient=" + xyStdDevCoefficient +
                    ", thetaStdDevCoefficient=" + thetaStdDevCoefficient +
                    ", maxTrustDistance=" + maxTrustDistance +
                    ", fieldBorderMargin=" + fieldBorderMargin +
                    ", zMargin=" + zMargin +
                    '}';
        }
    }

    protected static final class PoseEstimate {
        public final Pose3d cameraPose;
        public final double error;

        public PoseEstimate(Pose3d cameraPose, double error) {
            this.cameraPose = cameraPose;
            this.error = error;
        }

        @Override
        public String toString() {
            return "PoseEstimate{" +
                    "cameraPose=" + cameraPose +
                    ", error=" + error +
                    '}';
        }
    }

    protected static final class EstimateInput {
        public final double timestamp;
        public final PoseEstimate poseA; // Robot pose - not camera pose
        public final PoseEstimate poseB;
        public final int[] visibleTagIds;
        public final Translation2d[][] visibleTagCorners;

        public EstimateInput(double timestamp, PoseEstimate poseA, PoseEstimate poseB, int[] visibleTagIds, Translation2d[][] visibleTagCorners) {
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
                    ", visibleTagCorners=" + Arrays.toString(visibleTagCorners) +
                    '}';
        }
    }
}
