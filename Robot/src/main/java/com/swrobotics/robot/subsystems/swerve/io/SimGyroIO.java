package com.swrobotics.robot.subsystems.swerve.io;

public final class SimGyroIO implements GyroIO {
    @Override
    public void updateInputs(Inputs inputs) {
        // No gyro :(
        inputs.connected = false;
    }
}
