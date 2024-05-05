package com.swrobotics.robot.subsystems.tagtracker.io;

import com.swrobotics.robot.subsystems.tagtracker.CameraCaptureProperties;

public interface TagTrackerCameraIO {
    void updateInputs(Inputs inputs);

    void setCaptureProperties(CameraCaptureProperties props);

    final class Inputs {
        public long[] timestamps;
        public double[][] framePackedData;
    }
}
