package com.swrobotics.robot.subsystems.swerve.io;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.swrobotics.robot.logging.AutoLoggedInputs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class SwerveModuleIO {
    private final String name;

    public SwerveModuleIO(String name) {
        this.name = name;
    }

    public abstract void updateInputs(Inputs inputs);

    public abstract void setCANcoderMagnetOffset(double offset);

    public abstract void apply(SwerveModuleState state, SwerveModule.DriveRequestType driveRequestType);

    public String getName() {
        return name;
    }

    public static final class Inputs extends AutoLoggedInputs {
        public double drivePosition = 0;
        public double driveVelocity = 0;
        public double angle = 0;
        public double canCoderPos = 0;

        public SwerveModuleState getState() {
            return new SwerveModuleState(driveVelocity, Rotation2d.fromRotations(angle));
        }

        public SwerveModulePosition getPosition() {
            return new SwerveModulePosition(drivePosition, Rotation2d.fromRotations(angle));
        }
    }
}
