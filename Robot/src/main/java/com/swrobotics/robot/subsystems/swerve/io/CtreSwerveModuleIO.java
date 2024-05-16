package com.swrobotics.robot.subsystems.swerve.io;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.subsystems.music.MusicSubsystem;
import com.swrobotics.robot.subsystems.temperature.TemperatureTrackerSubsystem;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

// Swerve module with TalonFX motors and CanCoder
public final class CtreSwerveModuleIO extends SwerveModuleIO {
    private final SwerveModule module;
    private final StatusSignal<Double> canCoderPosition;

    public CtreSwerveModuleIO(String name, SwerveModuleConstants constants, String canBus) {
        super(name);
        module = new SwerveModule(constants, canBus);

        CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
        limits.SupplyCurrentLimitEnable = true;
        limits.SupplyCurrentLimit = Constants.kDriveCurrentLimit;
        limits.SupplyTimeThreshold = Constants.kDriveCurrentLimitTime;
        limits.StatorCurrentLimitEnable = false;
        module.getDriveMotor().getConfigurator().apply(limits);

        canCoderPosition = module.getCANcoder().getAbsolutePosition();

        MusicSubsystem.getInstance().addInstrument(module.getDriveMotor());
        MusicSubsystem.getInstance().addInstrument(module.getSteerMotor());
        TemperatureTrackerSubsystem.getInstance().addMotor(name + " Drive", canBus, module.getDriveMotor());
        TemperatureTrackerSubsystem.getInstance().addMotor(name + " Steer", canBus, module.getSteerMotor());

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.kPeriodicFreq,
                module.getDriveMotor().getPosition(),
                module.getDriveMotor().getVelocity(),
                module.getSteerMotor().getPosition(),
                module.getSteerMotor().getVelocity(),
                canCoderPosition
        );
        ParentDevice.optimizeBusUtilizationForAll(module.getDriveMotor(), module.getSteerMotor(), module.getCANcoder());
    }

    @Override
    public void updateInputs(Inputs inputs) {
        canCoderPosition.refresh();
        SwerveModulePosition position = module.getPosition(true);
        SwerveModuleState state = module.getCurrentState();

        inputs.drivePosition = position.distanceMeters;
        inputs.driveVelocity = state.speedMetersPerSecond;
        // Position's angle is latency compensated, so use it instead of state's angle
        inputs.angle = position.angle.getRotations();
        inputs.canCoderPos = canCoderPosition.getValue();
    }

    @Override
    public void setCANcoderMagnetOffset(double offset) {
        MagnetSensorConfigs configs = new MagnetSensorConfigs();
        configs.MagnetOffset = offset;
        module.getCANcoder().getConfigurator().apply(configs);
    }

    @Override
    public void apply(SwerveModuleState state, SwerveModule.DriveRequestType driveRequestType) {
        module.apply(state, driveRequestType, SwerveModule.SteerRequestType.MotionMagic);
    }
}
