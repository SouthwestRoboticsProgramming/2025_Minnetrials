package com.swrobotics.lib.input;

import com.swrobotics.lib.utils.MathUtil;

import java.util.function.Supplier;

/**
 * Represents an analog input. Most axes will have an output from either -1 to 1, or 0 to 1. Note
 * that most analog inputs have a slight "drift", where the value will not be exactly zero when not
 * touching the input.
 */
public final class InputAxis implements InputElement {
    private final Supplier<Double> getter;
    private final double deadband;
    private double value;

    /**
     * Creates a new input axis that reads its value from a given getter function.
     *
     * @param getter value getter
     */
    public InputAxis(Supplier<Double> getter, double deadband) {
        this.getter = getter;
        this.deadband = deadband;
        value = getter.get();
    }

    /**
     * Gets the current position of this axis with deadband applied.
     *
     * @return deadbanded value
     */
    public double get() {
        return MathUtil.deadband(value, deadband);
    }

    /**
     * Gets the current position of this axis without deadband applied.
     *
     * @return raw value
     */
    public double getRaw() {
        return value;
    }

    /**
     * @param range range to check, centered around 0
     * @return whether the current position is outside of the range
     */
    public boolean isOutside(double range) {
        return Math.abs(value) > range;
    }

    @Override
    public void update() {
        value = getter.get();
    }
}
