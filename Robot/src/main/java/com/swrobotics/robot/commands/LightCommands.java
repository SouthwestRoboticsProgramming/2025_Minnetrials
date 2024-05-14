package com.swrobotics.robot.commands;

import com.swrobotics.robot.subsystems.lights.LightsSubsystem;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;

public final class LightCommands {
    public static Command blink(LightsSubsystem lights, int blinks, Color color, double totalTimeSeconds, double onPercent) {
        if (onPercent < 0 || onPercent > 1)
            throw new IllegalArgumentException("onPercent must be between 0 and 1");

        if (onPercent == 0)
            return Commands.none();
        if (onPercent == 1)
            return setColorForTime(lights, color, totalTimeSeconds);

        double onTime = totalTimeSeconds / blinks * onPercent;
        double offTime = totalTimeSeconds / blinks - onTime;

        SequentialCommandGroup sequence = new SequentialCommandGroup();
        for (int i = 0; i < blinks; i++) {
            sequence.addCommands(
                    setColorForTime(lights, color, onTime),
                    new WaitCommand(offTime)
            );
        }

        return sequence;
    }

    public static Command blink(LightsSubsystem lights, int blinks, Color color) {
        return blink(lights, blinks, color, 0.5, 0.5);
    }

    public static Command blink(LightsSubsystem lights, Color color) {
        return blink(lights, 3, color);
    }

    public static Command setColorForTime(LightsSubsystem lights, Color color, double seconds) {
        return Commands.run(() -> lights.setCommandRequest(color)).withTimeout(seconds);
    }
}
