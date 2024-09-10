package com.swrobotics.lib.input;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/** Represents a binary input (pressed or not pressed). */
public final class InputButton implements InputElement {
    private final Supplier<Boolean> getter;
    private final List<Runnable> onPressed, onReleased;
    private boolean down, wasDown;

    /**
     * Creates a new input button that reads its value from a provided getter
     * function.
     *
     * @param getter value getter
     */
    public InputButton(Supplier<Boolean> getter) {
        this.getter = getter;

        onPressed = new ArrayList<>();
        onReleased = new ArrayList<>();

        down = wasDown = getter.get();
    }

    /**
     * @return whether this button is currently pressed
     */
    public boolean isDown() {
        return down;
    }

    /**
     * Gets whether this button was just pressed during this periodic cycle.
     * This is when the button was not down the previous periodic, but is now
     * down.
     *
     * @return if button was just pressed
     */
    public boolean wasPressed() {
        return down && !wasDown;
    }

    /**
     * Gets whether this button was just released during this periodic cycle.
     * This is when the button was down the previous periodic, but is now not
     * down.
     *
     * @return if button was just pressed
     */
    public boolean wasReleased() {
        return !down && wasDown;
    }

    /**
     * Adds a function that will be called whenever the button is pressed. This
     * function will be invoked on each periodic where {@link #wasPressed()}
     * returns {@code true}.
     *
     * @param pressedFn function to call
     * @return this
     */
    public InputButton onPressed(Runnable pressedFn) {
        onPressed.add(pressedFn);
        return this;
    }

    /**
     * Adds a command that will be called whenever the button is pressed. This
     * command will be scheduled on each periodic where {@link #wasPressed()}
     * returns {@code true}.
     *
     * @param command command to schedule
     * @return this
     */
    public InputButton onPressed(Command command) {
        onPressed(() -> CommandScheduler.getInstance().schedule(command));
        return this;
    }

    /**
     * Adds a function that will be called whenever the button is released.
     * This function will be invoked on each periodic where
     * {@link #wasReleased()} returns {@code true}.
     *
     * @param releasedFn function to call
     * @return this
     */
    public InputButton onReleased(Runnable releasedFn) {
        onReleased.add(releasedFn);
        return this;
    }

    /**
     * Adds a command that will be called whenever the button is released. This
     * command will be scheduled on each periodic where {@link #wasReleased()}
     * returns {@code true}.
     *
     * @param command command to schedule
     * @return this
     */
    public InputButton onReleased(Command command) {
        onReleased(() -> CommandScheduler.getInstance().schedule(command));
        return this;
    }

    /**
     * Adds a command that will be called whenever the button is held down for
     * a specified amount of time.
     *
     * @param command command to schedule
     * @param seconds time the button has to be held
     * @return this
     */
    public InputButton onHeld(Command command, double seconds) {
        new Trigger(this::isDown).debounce(seconds).onTrue(command);
        return this;
    }

    /**
     * Adds a command that will be called whenever the button is held down for
     * one second.
     *
     * @param command command to schedule
     * @return this
     */
    public InputButton onHeld(Command command) {
        return onHeld(command, 1.0);
    }

    @Override
    public void update() {
        wasDown = down;
        down = getter.get();
        if (wasPressed()) {
            for (Runnable pressedFn : onPressed) {
                pressedFn.run();
            }
        }
        if (wasReleased()) {
            for (Runnable releasedFn : onReleased) {
                releasedFn.run();
            }
        }
    }
}
