package com.swrobotics.robot.subsystems.swerve.io;

import com.swrobotics.robot.logging.AutoLoggedInputs;

public interface GyroIO {
    final class Inputs extends AutoLoggedInputs {
        public boolean connected;
        public double yaw; // Radians
    }

    void updateInputs(Inputs inputs);
}
