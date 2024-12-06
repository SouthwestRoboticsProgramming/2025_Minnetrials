package com.swrobotics.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.robot.config.Constants;

import com.swrobotics.robot.subsystems.motortracker.MotorTrackerSubsystem;
import com.swrobotics.robot.subsystems.music.MusicSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    public enum State {
        OFF(Constants.kIntakeIdleSpeed, Constants.kIntakeIdleSpeed),
        INTAKE(Constants.kIntakeSpeed, Constants.kIntakeSpeed),
        EJECT(Constants.kIntakeEjectSpeedTop, Constants.kIntakeEjectSpeedBottom);

        final NTEntry<Double> topSpeed, bottomSpeed;

        State(NTEntry<Double> topSpeed, NTEntry<Double> bottomSpeed) {
            this.topSpeed = topSpeed;
            this.bottomSpeed = bottomSpeed;
        }
    }
    
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

        MotorTrackerSubsystem.getInstance().addMotor("Intake 1", Motor1);
        MotorTrackerSubsystem.getInstance().addMotor("Intake 2", Motor2);
        MusicSubsystem.getInstance().addInstrument(Motor1);
        MusicSubsystem.getInstance().addInstrument(Motor2);
    }
    
//    public void move(double top, double bottom) {
//        Motor1.set(top);
//        Motor2.set(bottom);
//    }

    public void setState(State state) {
        Motor1.set(state.topSpeed.get());
        Motor2.set(state.bottomSpeed.get());
    }

    public boolean hasPiece() {
        statorCurrent.refresh();
        return statorCurrent.getValueAsDouble() > Constants.kIntakeStopCurrent.get();
    }
}
