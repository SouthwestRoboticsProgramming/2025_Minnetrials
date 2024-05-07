package com.swrobotics.robot.subsystems.swerve;

import com.swrobotics.robot.config.Constants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public final class SwerveKinematics {
    private final SwerveDriveKinematics kinematics;
    private final double maxVelocity;

    public SwerveKinematics(Translation2d[] modulePositions, double maxVelocity) {
        kinematics = new SwerveDriveKinematics(modulePositions);
        this.maxVelocity = maxVelocity;
    }

    public SwerveModuleState[] getStates(ChassisSpeeds robotRelSpeeds) {
        ChassisSpeeds discrete = ChassisSpeeds.discretize(robotRelSpeeds, Constants.kDriveDriftComp);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(discrete);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxVelocity);

        return states;
    }

    public Twist2d getTwistDelta(SwerveModulePosition[] startPositions, SwerveModulePosition[] endPositions) {
        SwerveModulePosition[] deltas = new SwerveModulePosition[startPositions.length];
        for (int i = 0; i < deltas.length; i++) {
            deltas[i] = new SwerveModulePosition(
                    endPositions[i].distanceMeters - startPositions[i].distanceMeters,
                    endPositions[i].angle);
        }

        return kinematics.toTwist2d(deltas);
    }

    public ChassisSpeeds toChassisSpeeds(SwerveModuleState[] moduleStates) {
        return kinematics.toChassisSpeeds(moduleStates);
    }
}
