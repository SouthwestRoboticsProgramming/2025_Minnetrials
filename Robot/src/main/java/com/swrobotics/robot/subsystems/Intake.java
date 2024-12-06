package com.swrobotics.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.robot.config.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    private final StatusSignal<Double> statorCurrent;

    //assign value to kraken
    TalonFX Motor1 = new TalonFX(5);
    TalonFX Motor2 = new TalonFX(6);

    public Intake() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // config.CurrentLimits.StatorCurrentLimitEnable = true;
        // config.CurrentLimits.StatorCurrentLimit = Constants.kIntakeCurrentLimit.get();

        Motor1.getConfigurator().apply(config);
        Motor2.getConfigurator().apply(config);

        statorCurrent = Motor1.getStatorCurrent();

        // Constants.kIntakeCurrentLimit.onChange((limit) -> {
        //     CurrentLimitsConfigs conf = new CurrentLimitsConfigs();
        //     conf.StatorCurrentLimitEnable = true;
        //     conf.StatorCurrentLimit = limit;

        //     Motor1.getConfigurator().apply(conf);
        //     Motor2.getConfigurator().apply(conf);
        // });
    }
    
    public void move(double top, double bottom) {
        Motor1.set(top);
        Motor2.set(bottom);
    }

    public boolean hasPiece() {
        statorCurrent.refresh();
        return statorCurrent.getValueAsDouble() > Constants.kIntakeStopCurrent.get();
    }
}
