package com.swrobotics.robot.config;

import com.swrobotics.lib.field.FieldInfo;
import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.robot.subsystems.vision.RawAprilTagSource;
import com.swrobotics.robot.subsystems.vision.tagtracker.TagTrackerCaptureProperties;

// Use NTEntry when you want tunable
// Use double when value has been tuned in so it can't accidentally change
public final class Constants {
    public static final int kPeriodicFreq = 50; // Hz
    public static final double kPeriodicTime = 1.0 / kPeriodicFreq;

    // We don't know what the 2025 field is yet :(
    public static final FieldInfo kField = FieldInfo.MINNETRIALS_BEACH_BASH_2025;
    public static final int kEndgameAlertTime = 30;

    // Controls
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final double kDeadband = 0.15;
    public static final double kTriggerThreshold = 0.3;

    // Pathfinding
    public static final String kPathfindingJson = "crescendo_pathfinding.json";
    public static final double kRobotRadius = 0.6202230647076; // m
    public static final double kPathfindingTolerance = 0.2; // m

    // Vision
    public static final String kAprilTagJson = "beach_bash_apriltag.json";

    // These need to be tuned during field calibration time at every event.
    // They can be adjusted manually in NetworkTables at TagTracker/<Camera>/Config/...
    // Don't forget to set your adjusted values here! The NetworkTables values
    // do not save.
    public static final TagTrackerCaptureProperties kTagTrackerCaptureProps = new TagTrackerCaptureProperties()
            .setAutoExposure(false)
            .setExposure(20)
            .setGain(1)
            .setTargetFps(50);

    public static final RawAprilTagSource.FilterParameters kTagTrackerFilterParams = new RawAprilTagSource.FilterParameters()
            .setAmbiguityThreshold(0.9)
            .setXYStdDevCoefficient(0.01)
            .setThetaStdDevCoefficient(0.01)
            .setFieldBorderMargin(0.5)
            .setZMargin(0.75)
            .setMaxTrustDistance(Double.POSITIVE_INFINITY);

    public static final double[] kVisionStateStdDevs = {0.005, 0.005, 0.001};
    public static final double kVisionHistoryTime = 0.3; // Secs
    // Time at beginning of teleop where vision angle is trusted more
    // Only applies when NOT on competition field!
    public static final double kVisionInitialTrustTime = 5; // Secs
    public static final double kVisionInitialAngleStdDev = 0.2;

    // Lights
    public static final int kLedStripLength = 22;
    public static final int kLowBatteryThreshold = 10; // Volts
    public static final int kLedCurrentShutoffThreshold = 250; // Amps

    // Motor tracking
    public static final double kMotorTrackInterval = 2; // Seconds
    public static final double kOverheatingThreshold = 75; // Celsius

    // This must be at the bottom of the file so it happens last
    static {
        NTEntry.cleanPersistent();
    }
}
