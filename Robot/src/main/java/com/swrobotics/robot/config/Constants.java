package com.swrobotics.robot.config;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.swrobotics.lib.field.FieldInfo;
import com.swrobotics.lib.net.NTDouble;
import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.robot.subsystems.swerve.SwerveModuleInfo;
import com.swrobotics.robot.subsystems.tagtracker.CameraCaptureProperties;
import edu.wpi.first.math.util.Units;

// Use NTEntry when you want tunable
// Use double when value has been tuned in so it can't accidentally change
// TODO: We might want to come up with a different solution than persistent values
//  since persistent values never go away
public final class Constants {
    // We don't know what the 2025 field is yet :(
    public static final FieldInfo kField = FieldInfo.CRESCENDO_2024;

    // Controls
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final double kDeadband = 0.15;
    public static final double kTriggerThreshold = 0.3;

    public static final double kDriveControlMaxAccel = 5000.5; // m/s^2
    public static final double kDriveControlMaxTurnSpeed = 1; // rot/s
    public static final double kDriveControlDrivePower = 2; // Exponent input is raised to
    public static final double kDriveControlTurnPower = 2;

    // Auto (TODO: Tune)
    public static final double kAutoDriveKp = 8;
    public static final double kAutoDriveKd = 0;

    public static final NTEntry<Double> kAutoTurnKp = new NTDouble("Auto/Turn PID/kP", 9).setPersistent();
    public static final NTEntry<Double> kAutoTurnKd = new NTDouble("Auto/Turn PID/kD", 0.5).setPersistent();

    // Drive
    private static final double kHalfSpacingX = 55.3 / 100 / 2; // m
    private static final double kHalfSpacingY = 63.0 / 100 / 2; // m
    public static final double kWheelRadius = 1.9; // Inches

    public static final NTEntry<Double> kFrontLeftOffset = new NTDouble("Drive/Modules/Front Left Offset (rot)", 0).setPersistent();
    public static final NTEntry<Double> kFrontRightOffset = new NTDouble("Drive/Modules/Front Right Offset (rot)", 0).setPersistent();
    public static final NTEntry<Double> kBackLeftOffset = new NTDouble("Drive/Modules/Back Left Offset (rot)", 0).setPersistent();
    public static final NTEntry<Double> kBackRightOffset = new NTDouble("Drive/Modules/Back Right Offset (rot)", 0).setPersistent();
    public static final SwerveModuleInfo[] kSwerveModuleInfos = {
            new SwerveModuleInfo(IOAllocation.CAN.SWERVE_FL, kHalfSpacingX, kHalfSpacingY, Constants.kFrontLeftOffset, "Front Left"),
            new SwerveModuleInfo(IOAllocation.CAN.SWERVE_FR, kHalfSpacingX, -kHalfSpacingY, Constants.kFrontRightOffset, "Front Right"),
            new SwerveModuleInfo(IOAllocation.CAN.SWERVE_BL, -kHalfSpacingX, kHalfSpacingY, Constants.kBackLeftOffset, "Back Left"),
            new SwerveModuleInfo(IOAllocation.CAN.SWERVE_BR, -kHalfSpacingX, -kHalfSpacingY, Constants.kBackRightOffset, "Back Right")
    };

    public static final double kDriveRadius = Math.hypot(kHalfSpacingX, kHalfSpacingY);
    public static final double kMaxAchievableSpeed = Units.feetToMeters(18.9); // m/s

    public static final double kDriveDriftComp = 0.04; // dt for chassis speeds discretize

    public static final double kDriveCurrentLimit = 40; // A
    public static final double kDriveCurrentLimitTime = 0.25; // sec

    private static final Slot0Configs kSteerGains = new Slot0Configs()
            .withKP(100).withKD(0.05);
    private static final Slot0Configs kDriveGains = new Slot0Configs()
            .withKP(3).withKD(0);
    private static final double kDriveSlipCurrent = 300; // A

    private static final double kSwerveCoupleRatio = 50.0 / 16;
    private static final double kDriveGearRatio = (50.0/16) * (16.0/28) * (45.0/15);
    private static final double kSteerGearRatio = 150.0 / 7;

    private static final boolean kSteerMotorReversed = true;

    public static final SwerveModuleConstantsFactory kSwerveConstantsFactory = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadius)
            .withSlipCurrent(kDriveSlipCurrent)
            .withSteerMotorGains(kSteerGains)
            .withDriveMotorGains(kDriveGains)
            .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
            .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
            .withSpeedAt12VoltsMps(kMaxAchievableSpeed)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(kSwerveCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed);

    // Vision
    public static final CameraCaptureProperties kVisionCaptureProps = new CameraCaptureProperties()
            .setAutoExposure(false)
            .setExposure(20)
            .setGain(1)
            .setTargetFps(50);

    public static final double kVisionAmbiguityThreshold = 0.9;
    public static final double kVisionFieldBorderMargin = 0.5;
    public static final double kVisionZMargin = 0.75;
    public static final double kVisionXYStdDevCoeff = 0.01;
    public static final double kVisionThetaStdDevCoeff = 0.01;

    public static final double[] kVisionStateStdDevs = {0.005, 0.005, 0.001};
    public static final double kVisionHistoryTime = 0.3; // Secs
    // Time at beginning of teleop where vision angle is trusted more
    // Only applies when NOT on competition field!
    public static final double kVisionInitialTrustTime = 5; // Secs
    public static final double kVisionInitialAngleStdDev = 0.2;

    // Lights
    public static final int kLedStripLength = 22;
    public static final int kLowBatteryThreshold = 10; // Volts
}
