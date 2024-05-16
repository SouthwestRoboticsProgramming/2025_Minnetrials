package com.swrobotics.robot.subsystems.temperature;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public final class RealTemperatureIO implements TemperatureIO {
    private record TrackedMotor(String name, String canBus, StatusSignal<Double> tempStatus) {}

    private final List<TrackedMotor> motors = new ArrayList<>();
    private BaseStatusSignal[][] allSignals = null;

    @Override
    public void updateInputs(Inputs inputs) {
        int count = motors.size();

        // Only allocate arrays once
        if (allSignals == null) {
            Map<String, List<BaseStatusSignal>> groups = new HashMap<>();
            for (TrackedMotor motor : motors) {
                List<BaseStatusSignal> group = groups.computeIfAbsent(motor.canBus, (b) -> new ArrayList<>());
                group.add(motor.tempStatus);
            }

            allSignals = new BaseStatusSignal[groups.size()][];
            int groupIdx = 0;
            for (List<BaseStatusSignal> group : groups.values()) {
                BaseStatusSignal[] signals = group.toArray(new BaseStatusSignal[0]);
                allSignals[groupIdx++] = signals;
            }

            inputs.names = new String[count];
            inputs.temperatures = new double[count];
        }

        // Refresh all status signals grouped by CAN bus
        for (BaseStatusSignal[] group : allSignals) {
            BaseStatusSignal.refreshAll(group);
        }

        for (int i = 0; i < count; i++) {
            TrackedMotor motor = motors.get(i);
            inputs.names[i] = motor.name();
            inputs.temperatures[i] = motor.tempStatus().getValue();
        }
    }

    @Override
    public void addMotor(String name, String canBus, TalonFX motor) {
        if (allSignals != null)
            throw new IllegalStateException("Can only add motors in robotInit()");

        StatusSignal<Double> tempStatus = motor.getDeviceTemp();
        tempStatus.setUpdateFrequency(4); // 4 Hz is minimum update frequency supported
        motors.add(new TrackedMotor(name, canBus, tempStatus));
    }
}
