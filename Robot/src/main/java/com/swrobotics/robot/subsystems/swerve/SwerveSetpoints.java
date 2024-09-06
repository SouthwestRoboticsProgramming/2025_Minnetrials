package com.swrobotics.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public final class SwerveSetpoints {
    public static SwerveSetpoints createInitial(int moduleCount) {
        SwerveModuleState[] states = new SwerveModuleState[moduleCount];
        for (int i = 0; i < moduleCount; i++) {
            states[i] = new SwerveModuleState();
        }
        return new SwerveSetpoints(new ChassisSpeeds(), null, states);
    }

    public ChassisSpeeds chassisSpeeds;
    public SwerveModuleState[] desiredModuleStates; // The eventual target state
    public SwerveModuleState[] moduleStates; // The setpoints for this periodic

    public SwerveSetpoints(ChassisSpeeds chassisSpeeds, SwerveModuleState[] desiredModuleStates, SwerveModuleState[] initialStates) {
        this.chassisSpeeds = chassisSpeeds;
        this.desiredModuleStates = desiredModuleStates;
        this.moduleStates = initialStates;
    }
}
