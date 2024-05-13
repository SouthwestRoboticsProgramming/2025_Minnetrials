package com.swrobotics.robot.subsystems.temperature;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import java.util.ArrayList;
import java.util.List;

public final class RealTemperatureIO implements TemperatureIO {
    private record TrackedMotor(String name, StatusSignal<Double> tempStatus) {}

    private final List<TrackedMotor> motors = new ArrayList<>();
    private BaseStatusSignal[] allSignals;

    @Override
    public void updateInputs(Inputs inputs) {
        int count = motors.size();

        // Only allocate arrays once to minimize performance impact
        if (allSignals == null) {
            allSignals = new BaseStatusSignal[count];
            for (int i = 0; i < count; i++)
                allSignals[i] = motors.get(i).tempStatus;

            // Assume we'll always receive the same Inputs object, which should
            // always be the case
            inputs.names = new String[count];
            inputs.temperatures = new double[count];
        }

        // Refresh all at once so it takes less time. We don't want to take
        // long here so we don't disturb the main robot loop timing
        BaseStatusSignal.refreshAll(allSignals);

        for (int i = 0; i < count; i++) {
            TrackedMotor motor = motors.get(i);
            inputs.names[i] = motor.name();
            inputs.temperatures[i] = motor.tempStatus().getValue();
        }
    }

    @Override
    public void addMotor(String name, TalonFX motor) {
        if (allSignals != null)
            throw new IllegalStateException("Can only add motors in robotInit()");
        motors.add(new TrackedMotor(name, motor.getDeviceTemp()));
    }
}
