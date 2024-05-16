package com.swrobotics.robot.subsystems.vision.tagtracker;

import com.swrobotics.robot.logging.AutoLoggedInputs;

public interface TagTrackerEnvironmentIO {
    void updateInputs(Inputs inputs);

    final class Inputs extends AutoLoggedInputs {
        public boolean dataChanged = false;
        public double[] packedData = new double[0];
    }
}
