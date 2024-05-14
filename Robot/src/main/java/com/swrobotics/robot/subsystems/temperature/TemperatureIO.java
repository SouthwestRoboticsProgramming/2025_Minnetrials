package com.swrobotics.robot.subsystems.temperature;

import com.ctre.phoenix6.hardware.TalonFX;
import com.swrobotics.robot.logging.AutoLoggedInputs;

public interface TemperatureIO {
    final class Inputs extends AutoLoggedInputs {
        public String[] names = new String[0];
        public double[] temperatures = new double[0];
    }

    void updateInputs(Inputs inputs);

    void addMotor(String name, String canBus, TalonFX motor);
}
