package com.swrobotics.robot.subsystems.swerve.io;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public final class NavXGyroIO implements GyroIO {
    private final AHRS navx;

    public NavXGyroIO() {
        navx = new AHRS(SPI.Port.kMXP);
    }

    @Override
    public void updateInputs(Inputs inputs) {
        inputs.connected = navx.isConnected();
        if (inputs.connected)
            inputs.yaw = navx.getRotation2d().getRadians();
    }
}
