package com.swrobotics.robot.subsystems.vision.tagtracker;

import com.swrobotics.robot.subsystems.vision.tagtracker.TagTrackerCaptureProperties;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface TagTrackerCameraIO {
    void updateInputs(Inputs inputs);

    void setCaptureProperties(TagTrackerCaptureProperties props);

    final class Inputs implements LoggableInputs {
        // false value to indicate no estimate
        private static final byte[] EMPTY_DATA = {0};

        public long[] timestamps;
        public byte[][] framePackedData;

        @Override
        public void toLog(LogTable table) {
            table.put("timestamps", timestamps);
            table.put("frameCount", framePackedData.length);
            for (int i = 0; i < framePackedData.length; i++) {
                byte[] frameData = framePackedData[i];
                table.put("frames/" + i, frameData);
            }
        }

        @Override
        public void fromLog(LogTable table) {
            timestamps = table.get("timestamps", new long[0]);
            int frameCount = table.get("frameCount", 0);
            framePackedData = new byte[frameCount][];
            for (int i = 0; i < frameCount; i++) {
                framePackedData[i] = table.get("frames/" + i, EMPTY_DATA);
            }
        }
    }
}
