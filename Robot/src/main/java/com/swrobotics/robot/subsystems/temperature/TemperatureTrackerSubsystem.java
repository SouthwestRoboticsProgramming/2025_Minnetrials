package com.swrobotics.robot.subsystems.temperature;

import com.ctre.phoenix6.hardware.TalonFX;
import com.swrobotics.robot.config.Constants;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem to track the temperature of all the motors in the robot to detect
 * if something is overheating.
 */
public final class TemperatureTrackerSubsystem extends SubsystemBase {
    private static TemperatureTrackerSubsystem instance = null;

    public static TemperatureTrackerSubsystem getInstance() {
        if (instance == null)
            throw new IllegalStateException("TemperatureTrackerSubsystem is not yet initialized");
        return instance;
    }

    private final TemperatureIO io;
    private final TemperatureIO.Inputs inputs;
    private final Timer periodicTimer;
    private boolean overheating;

    public TemperatureTrackerSubsystem() {
        if (instance != null)
            throw new IllegalStateException("TemperatureTrackerSubsystem already initialized");
        instance = this;

        if (RobotBase.isReal())
            io = new RealTemperatureIO();
        else
            io = new SimTemperatureIO();
        inputs = new TemperatureIO.Inputs();

        periodicTimer = new Timer();
        periodicTimer.start();

        overheating = false;
    }

    /**
     * Adds a motor to be monitored.
     *
     * @param name name of the motor for logging
     * @param canBus CAN bus the motor is attached to
     * @param motor motor to add
     */
    public void addMotor(String name, String canBus, TalonFX motor) {
        io.addMotor(name, canBus, motor);
    }

    @Override
    public void periodic() {
        // Only sample temperature every so often
        if (!periodicTimer.advanceIfElapsed(Constants.kTemperatureInterval))
            return;

        io.updateInputs(inputs);
        Logger.processInputs("Temperatures", inputs);

        overheating = false;
        for (double temp : inputs.temperatures) {
            if (temp > Constants.kOverheatingThreshold) {
                overheating = true;
                break;
            }
        }

        Logger.recordOutput("Overheating", overheating);
    }

    public boolean isOverheating() {
        return overheating;
    }
}
