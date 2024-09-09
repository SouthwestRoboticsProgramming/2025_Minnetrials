package com.swrobotics.robot.subsystems.swerve;

/**
 * Limits for the components of each swerve module.
 */
public final class SwerveKinematicLimits {
    /** Maximum drive velocity (m/s) */
    public double kMaxDriveVelocity;

    /**
     * Maximum module drive acceleration (m/s^2). This is not necessarily the
     * same as the drive base maximum acceleration!
     */
    public double kMaxDriveAcceleration;

    /** Maximum steering velocity (rad/s) */
    public double kMaxSteeringVelocity;
}
