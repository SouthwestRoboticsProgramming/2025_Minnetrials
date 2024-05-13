package com.swrobotics.robot.subsystems.temperature;

import com.ctre.phoenix6.hardware.TalonFX;
import com.swrobotics.robot.config.Constants;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public final class TemperatureTrackerSubsystem extends SubsystemBase {
    private static TemperatureTrackerSubsystem instance = new TemperatureTrackerSubsystem();

    public static TemperatureTrackerSubsystem getInstance() {
        return instance;
    }

    private final TemperatureIO io;
    private final TemperatureIO.Inputs inputs;
    private final Timer periodicTimer;

    public TemperatureTrackerSubsystem() {
        instance = this;

        if (RobotBase.isReal())
            io = new RealTemperatureIO();
        else
            io = new SimTemperatureIO();
        inputs = new TemperatureIO.Inputs();

        periodicTimer = new Timer();
        periodicTimer.start();
    }

    public void addMotor(String name, TalonFX motor) {
        io.addMotor(name, motor);
    }

    @Override
    public void periodic() {
        if (!periodicTimer.advanceIfElapsed(Constants.kTemperatureInterval))
            return;

        // Only thing we have to do is log
        io.updateInputs(inputs);
        Logger.processInputs("Temperatures", inputs);
    }
}
