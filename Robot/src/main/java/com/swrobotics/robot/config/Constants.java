package com.swrobotics.robot.config;

import com.swrobotics.lib.field.FieldInfo;
import com.swrobotics.lib.net.NTDouble;
import com.swrobotics.lib.net.NTEntry;

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


    public static final NTEntry<Double> kArmKP = new NTDouble("Arm/kP", 30).setPersistent();
    public static final NTEntry<Double> kArmKD = new NTDouble("Arm/kD", 0).setPersistent();
    public static final NTEntry<Double> kArmKG = new NTDouble("Arm/kG", 0).setPersistent();

    public static final NTEntry<Double> kArmUpAngle = new NTDouble("Arm/Up Angle", 60).setPersistent();
    public static final NTEntry<Double> kArmDownAngle = new NTDouble("Arm/Down Angle", 4).setPersistent();

    public static final NTEntry<Double> kIntakeSpeed = new NTDouble("Intake/Intake Speed", 0.2).setPersistent();
    public static final NTEntry<Double> kIntakeEjectSpeedTop = new NTDouble("Intake/Eject Speed (Top)", -0.1).setPersistent();
    public static final NTEntry<Double> kIntakeEjectSpeedBottom = new NTDouble("Intake/Eject Speed (Bottom)", 1.0).setPersistent();
    public static final NTEntry<Double> kIntakeIdleSpeed = new NTDouble("Intake/Idle Speed", -0.02).setPersistent();
    public static final NTEntry<Double> kIntakeStopCurrent = new NTDouble("Intake/Stop Current", 10).setPersistent();


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
