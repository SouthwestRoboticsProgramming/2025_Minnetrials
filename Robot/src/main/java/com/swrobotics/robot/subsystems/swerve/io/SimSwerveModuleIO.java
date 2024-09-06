package com.swrobotics.robot.subsystems.swerve.io;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.swrobotics.lib.utils.MathUtil;
import com.swrobotics.robot.config.Constants;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public final class SimSwerveModuleIO extends SwerveModuleIO {
    private SwerveModuleState state;
    private double simDistance;
    private double magnetOffset;

    public SimSwerveModuleIO(String name, double magnetOffset) {
        super(name);

        state = new SwerveModuleState();
        simDistance = 0;
        this.magnetOffset = magnetOffset;
    }

    @Override
    public void updateInputs(Inputs inputs) {
        simDistance += state.speedMetersPerSecond * Constants.kPeriodicTime;

        inputs.angle = state.angle.getRotations();
        inputs.canCoderPos = MathUtil.wrap(inputs.angle + magnetOffset, 0, 1);
        inputs.driveVelocity = state.speedMetersPerSecond;
        inputs.drivePosition = simDistance;
    }

    @Override
    public void setCANcoderMagnetOffset(double offset) {
        magnetOffset = offset;
    }

    @Override
    public void apply(SwerveModuleState state, SwerveModule.DriveRequestType driveRequestType) {
        this.state = SwerveModuleState.optimize(state, this.state.angle);
    }
}
