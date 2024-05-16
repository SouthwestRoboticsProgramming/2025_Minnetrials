package com.swrobotics.robot.subsystems.vision.tagtracker;

public final class TagTrackerCaptureProperties {
    private boolean autoExposure = false;
    private double exposure = 30;
    private double gain = 1;
    private double targetFps = 50;

    public TagTrackerCaptureProperties setAutoExposure(boolean autoExposure) {
        this.autoExposure = autoExposure;
        return this;
    }

    public TagTrackerCaptureProperties setExposure(double exposure) {
        this.exposure = exposure;
        return this;
    }

    public TagTrackerCaptureProperties setGain(double gain) {
        this.gain = gain;
        return this;
    }

    public TagTrackerCaptureProperties setTargetFps(double targetFps) {
        this.targetFps = targetFps;
        return this;
    }

    public boolean isAutoExposure() {
        return autoExposure;
    }

    public double getExposure() {
        return exposure;
    }

    public double getGain() {
        return gain;
    }

    public double getTargetFps() {
        return targetFps;
    }
}
