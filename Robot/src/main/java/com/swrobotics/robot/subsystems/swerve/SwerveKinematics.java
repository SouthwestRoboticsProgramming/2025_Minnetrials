package com.swrobotics.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Conversions from ChassisSpeeds to module states and back the other way
 */
// TODO: Merge with SwerveSetpointGenerator?
public final class SwerveKinematics {
    private final SwerveDriveKinematics kinematics;

    /**
     * @param modulePositions positions of the modules relative to the center
     *                        of the drive base, in meters
     */
    public SwerveKinematics(Translation2d[] modulePositions) {
        kinematics = new SwerveDriveKinematics(modulePositions);
    }

    /**
     * Gets the resulting movement of the drive base from the module positions.
     *
     * @param startPositions positions of the modules before the movement
     * @param endPositions positions of the modules after the movement
     * @return overall drive base movement
     */
    public Twist2d getTwistDelta(SwerveModulePosition[] startPositions, SwerveModulePosition[] endPositions) {
        SwerveModulePosition[] deltas = new SwerveModulePosition[startPositions.length];
        for (int i = 0; i < deltas.length; i++) {
            deltas[i] = new SwerveModulePosition(
                    endPositions[i].distanceMeters - startPositions[i].distanceMeters,
                    endPositions[i].angle);
        }

        return kinematics.toTwist2d(deltas);
    }

    /**
     * Gets the resulting speeds of the drive base from module states.
     *
     * @param moduleStates states of the modules
     * @return speeds of the drive base that would result from those states
     */
    public ChassisSpeeds toChassisSpeeds(SwerveModuleState[] moduleStates) {
        return kinematics.toChassisSpeeds(moduleStates);
    }
}
