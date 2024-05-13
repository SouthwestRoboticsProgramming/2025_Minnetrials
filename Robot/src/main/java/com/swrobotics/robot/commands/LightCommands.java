package com.swrobotics.robot.commands;

import java.util.Optional;

import com.swrobotics.lib.utils.MathUtil;
import com.swrobotics.robot.RobotContainer;
import com.swrobotics.robot.subsystems.lights.LightsSubsystem;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LightCommands {

    public static Command blink(LightsSubsystem lights, int blinks, Color color, double totalTimeSeconds, double onPercent) {
        if (onPercent < 0 || onPercent > 1) {
            throw new IllegalArgumentException("onPercent must be between 0 and 1");
        }
        double onTime = totalTimeSeconds / blinks * onPercent;
        double offTime = totalTimeSeconds / blinks - onTime;
        Command sequence = new InstantCommand();
        for (int i = 0; i < blinks; i++) {
            sequence = sequence.andThen(setColorForTime(lights, color, onTime));
            sequence = sequence.andThen(new WaitCommand(offTime));
        }

        sequence = sequence.finallyDo(() -> lights.setOverride(Optional.empty()));

        return sequence;
    }

    public static Command blink(LightsSubsystem lights, int blinks, Color color) {
        return blink(lights, blinks, color, 0.5, 0.5);
    }

    public static Command blink(LightsSubsystem lights, Color color) {
        return blink(lights, 3, color);
    }

    private static Command setColor(LightsSubsystem lights, Color color) {
        return Commands.runOnce(() -> lights.setOverride(Optional.of(color)), lights);
    }

    public static Command setColorForTime(LightsSubsystem lights, Color color, double seconds) {
        return setColor(lights, color)
            .andThen(new WaitCommand(seconds))
            .andThen(Commands.runOnce(() ->lights.setOverride(Optional.empty())));
    }
}
