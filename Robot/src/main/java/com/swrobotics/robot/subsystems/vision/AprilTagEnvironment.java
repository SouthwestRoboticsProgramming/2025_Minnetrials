package com.swrobotics.robot.subsystems.vision;

import com.swrobotics.robot.subsystems.vision.tagtracker.NTEnvironmentIO;
import com.swrobotics.robot.subsystems.vision.tagtracker.TagTrackerEnvironmentIO;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayTopic;
import org.littletonrobotics.junction.Logger;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

// FIXME: This should not read from TagTracker, should read from JSON file
public final class AprilTagEnvironment {
    private final TagTrackerEnvironmentIO io;
    private final TagTrackerEnvironmentIO.Inputs inputs;

    private final Map<Integer, Pose3d> poses;

    public AprilTagEnvironment(DoubleArrayTopic topic) {
        io = new NTEnvironmentIO(topic);
        inputs = new TagTrackerEnvironmentIO.Inputs();

        poses = new HashMap<>();
    }

    // Returns pose if tag exists, else null
    public Pose3d getPose(int tagId) {
        return poses.get(tagId);
    }

    public Collection<Pose3d> getAllPoses() {
        return poses.values();
    }

    public void update() {
        io.updateInputs(inputs);
        Logger.processInputs("TagTracker/Environment", inputs);

        if (!inputs.dataChanged)
            return;
        double[] data = inputs.packedData;

        poses.clear();
        for (int i = 0; i < data.length; i += 8) {
            int tagId = (int) data[i];

            double tx = data[i + 1];
            double ty = data[i + 2];
            double tz = data[i + 3];
            double qw = data[i + 4];
            double qx = data[i + 5];
            double qy = data[i + 6];
            double qz = data[i + 7];

            Translation3d translation = new Translation3d(tx, ty, tz);
            Rotation3d rotation = new Rotation3d(new Quaternion(qw, qx, qy, qz));
            Pose3d pose = new Pose3d(translation, rotation);

            poses.put(tagId, pose);
        }
    }
}
