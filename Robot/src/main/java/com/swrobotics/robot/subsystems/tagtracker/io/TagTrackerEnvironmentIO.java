package com.swrobotics.robot.subsystems.tagtracker.io;

public interface TagTrackerEnvironmentIO {
    void updateInputs(Inputs inputs);

    final class Inputs {
        public boolean dataChanged = false;
        public double[] packedData = new double[0];
    }
}
