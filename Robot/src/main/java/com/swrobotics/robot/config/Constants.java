package com.swrobotics.robot.config;

import com.swrobotics.lib.field.FieldInfo;
import com.swrobotics.lib.net.NTDouble;
import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.robot.subsystems.swerve.SwerveModule;
import edu.wpi.first.math.util.Units;

// Use NTEntry when you want tunable
// Use double when value has been tuned in so it can't accidentally change
// TODO: We might want to come up with a different solution than persistent values
//  since persistent values never go away
public final class Constants {
    // We don't know what the 2025 field is yet :(
    public static final FieldInfo kField = FieldInfo.CRESCENDO_2024;

    // Controls
    public static final double kDeadband = 0.15;
    public static final double kTriggerThreshold = 0.3;

    public static final double kDriveControlMaxAccel = 5.5; // m/s^2
    public static final double kDriveControlMaxTurnSpeed = 1; // rot/s
    public static final double kDriveControlDrivePower = 2; // Exponent input is raised to
    public static final double kDriveControlTurnPower = 2;

    // Auto
    public static final NTEntry<Double> kAutoTurnKp = new NTDouble("Auto/Turn PID/kP", 9).setPersistent();
    public static final NTEntry<Double> kAutoTurnKd = new NTDouble("Auto/Turn PID/kD", 0.5).setPersistent();

    // Drive
    private static final double kHalfSpacingX = 55.3 / 100 / 2; // m
    private static final double kHalfSpacingY = 63.0 / 100 / 2; // m
    public static final SwerveModule.Info[] kSwerveModuleInfos = {
            new SwerveModule.Info(IOAllocation.CAN.SWERVE_FL, kHalfSpacingX, kHalfSpacingY, Constants.kFrontLeftOffset, "Front Left"),
            new SwerveModule.Info(IOAllocation.CAN.SWERVE_FR, kHalfSpacingX, -kHalfSpacingY, Constants.kFrontRightOffset, "Front Right"),
            new SwerveModule.Info(IOAllocation.CAN.SWERVE_BL, -kHalfSpacingX, kHalfSpacingY, Constants.kBackLeftOffset, "Back Left"),
            new SwerveModule.Info(IOAllocation.CAN.SWERVE_BR, -kHalfSpacingX, -kHalfSpacingY, Constants.kBackRightOffset, "Back Right")
    };
    public static final double kDriveRadius = Math.hypot(kHalfSpacingX, kHalfSpacingY);
    public static final double kMaxAchievableSpeed = Units.feetToMeters(18.9); // m/s

    public static final double kDriveCurrentLimit = 40; // A
    public static final double kDriveCurrentLimitTime = 0.25; // sec

    public static final NTEntry<Double> kFrontLeftOffset = new NTDouble("Drive/Modules/Front Left Offset (rot)", 0).setPersistent();
    public static final NTEntry<Double> kFrontRightOffset = new NTDouble("Drive/Modules/Front Right Offset (rot)", 0).setPersistent();
    public static final NTEntry<Double> kBackLeftOffset = new NTDouble("Drive/Modules/Back Left Offset (rot)", 0).setPersistent();
    public static final NTEntry<Double> kBackRightOffset = new NTDouble("Drive/Modules/Back Right Offset (rot)", 0).setPersistent();

    // Lights
    public static final int kLedStripLength = 22;
    public static final int kLowBatteryThreshold = 10; // Volts
}
