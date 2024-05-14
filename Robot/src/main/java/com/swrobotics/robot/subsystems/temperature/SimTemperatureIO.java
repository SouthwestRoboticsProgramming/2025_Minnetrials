package com.swrobotics.robot.subsystems.temperature;

import com.ctre.phoenix6.hardware.TalonFX;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public final class SimTemperatureIO implements TemperatureIO {
    // We don't have anything physical to measure, just assume room temp
    private static final double TEMP = 22;

    private final List<String> names = new ArrayList<>();

    @Override
    public void updateInputs(Inputs inputs) {
        inputs.names = names.toArray(names.toArray(new String[0]));
        inputs.temperatures = new double[names.size()];
        Arrays.fill(inputs.temperatures, TEMP + 5 * Math.random());
    }

    @Override
    public void addMotor(String name, String canBus, TalonFX motor) {
        names.add(name);
    }
}
