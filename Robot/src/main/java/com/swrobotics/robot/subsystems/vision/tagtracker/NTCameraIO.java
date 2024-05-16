package com.swrobotics.robot.subsystems.vision.tagtracker;

import edu.wpi.first.networktables.*;

public final class NTCameraIO implements TagTrackerCameraIO {
    private final RawSubscriber posesSub;

    private final BooleanPublisher autoExposurePub;
    private final DoublePublisher exposurePub;
    private final DoublePublisher gainPub;
    private final DoublePublisher targetFpsPub;

    public NTCameraIO(NetworkTable table) {
        posesSub = table.getSubTable("Outputs").getRawTopic("poses").subscribe(
                "raw",
                new byte[]{0},
                PubSubOption.sendAll(true),
                PubSubOption.keepDuplicates(true));

        NetworkTable configTable = table.getSubTable("Config");
        autoExposurePub = configTable.getBooleanTopic("Auto Exposure").publish();
        exposurePub = configTable.getDoubleTopic("Exposure").publish();
        gainPub = configTable.getDoubleTopic("Gain").publish();
        targetFpsPub = configTable.getDoubleTopic("Target FPS").publish();
    }

    @Override
    public void setCaptureProperties(TagTrackerCaptureProperties props) {
        autoExposurePub.set(props.isAutoExposure());
        exposurePub.set(props.getExposure());
        gainPub.set(props.getGain());
        targetFpsPub.set(props.getTargetFps());
    }

    @Override
    public void updateInputs(Inputs inputs) {
        TimestampedRaw[] data = posesSub.readQueue();
        inputs.timestamps = new long[data.length];
        inputs.framePackedData = new byte[data.length][];

        for (int i = 0; i < data.length; i++) {
            inputs.timestamps[i] = data[i].timestamp;
            inputs.framePackedData[i] = data[i].value;
        }
    }
}
