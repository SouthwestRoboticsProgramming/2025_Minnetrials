package com.swrobotics.lib.utils;

import edu.wpi.first.wpilibj.DriverStation;

public final class DriverStationUtil {
    private static double prevMatchTime = Double.NaN;
    private static boolean currentGuess = false;

    // Guesses whether the robot is in a real match based on match timer
    public static boolean isEnabledInMatchOrPractice() {
        if (!DriverStation.isEnabled()) {
            prevMatchTime = Double.NaN;
            currentGuess = false;
            return false;
        }

        double matchTime = DriverStation.getMatchTime();
        double diff = matchTime - prevMatchTime;
        if (Double.isNaN(prevMatchTime)) {
            prevMatchTime = matchTime;
            currentGuess = false;
            return false; // We don't know yet
        }

        // Timer counts down in real match and in DriverStation practice mode,
        // but up in manual DriverStation enabling
        boolean matchOrPractice = diff <= 0;

        // Only update guess once per time update
        if (prevMatchTime != matchTime) {
            prevMatchTime = matchTime;
            currentGuess = matchOrPractice;
        }
        return currentGuess;
    }
}
