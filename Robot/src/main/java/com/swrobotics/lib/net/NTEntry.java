package com.swrobotics.lib.net;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A data entry stored in NetworkTables. These are also logged to AdvantageKit
 * in case NetworkTables data is lost.
 *
 * @param <T> type of data stored in the entry
 */
public abstract class NTEntry<T> implements Supplier<T> {
    private static final String PREFIX = "NTEntry";
    private static final List<NTEntry<?>> ALL_ENTRIES = new ArrayList<>();

    /**
     * Updates all NTEntries. Should be called once per periodic.
     */
    public static void updateAll() {
        for (NTEntry<?> entry : ALL_ENTRIES) {
            entry.update();
        }
    }

    private final NetworkTableEntry entry;
    private final List<Consumer<T>> changeListeners;
    private final LoggableInputs inputs;

    private final T defaultValue;
    private T prevValue, value;

    protected abstract T getValue(NetworkTableEntry entry, T defaultValue);
    protected abstract void setValue(NetworkTableEntry entry, T value);

    // Despite all the get() and put() methods on LogTable having the same
    // name, they are different methods so we need to abstract them
    protected abstract void toLog(LogTable table, String key, T value);
    protected abstract T fromLog(LogTable table, String key, T defaultValue);

    /**
     * Creates a new entry with a specified path. The path can be split using the '/' character to
     * organize entries into groups.
     *
     * @param path path
     * @param defaultValue value to set if no value is present
     */
    public NTEntry(String path, T defaultValue) {
        this.defaultValue = defaultValue;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("");
        String[] parts = path.split("/");
        for (int i = 0; i < parts.length - 1; i++) {
            table = table.getSubTable(parts[i]);
        }
        entry = table.getEntry(parts[parts.length - 1]);

        if (!entry.exists())
            setValue(entry, defaultValue);
        prevValue = value = getValue(entry, defaultValue);

        changeListeners = new ArrayList<>();
        inputs = new LoggableInputs() {
            @Override
            public void toLog(LogTable table) {
                NTEntry.this.toLog(table, path, value);
            }

            @Override
            public void fromLog(LogTable table) {
                value = NTEntry.this.fromLog(table, path, defaultValue);
            }
        };

        ALL_ENTRIES.add(this);
    }

    @Override
    public T get() {
        return value;
    }

    /**
     * Sets the value stored within the entry. This will call change listeners
     * if the value is different than the current value.
     *
     * @param value value to set
     */
    public void set(T value) {
        this.value = value;
    }

    /**
     * Marks this entry to be stored if the robot is turned off. The value is
     * stored on the RoboRIO, so it is lost if the RoboRIO is re-imaged! You
     * can back up the persistent values by copying the
     * {@code /home/lvuser/networktables.json} file from the RoboRIO to
     * somewhere safe.
     *
     * @return this
     */
    public NTEntry<T> setPersistent() {
        entry.setPersistent();

        if (!Objects.equals(defaultValue, get()))
            DriverStation.reportWarning("NT value differs from default: " + entry.getName(), false);

        return this;
    }

    /**
     * Registers a change listener. The function provided will be called
     * whenever the value within the entry changes.
     *
     * @param listener listener to register
     */
    public void onChange(Consumer<T> listener) {
        changeListeners.add(listener);
    }

    /**
     * Registers a change listener and also calls it immediately. The function
     * provided will be called whenever the value within the entry changes.
     *
     * @param listener listener to register
     */
    public void nowAndOnChange(Consumer<T> listener) {
        listener.accept(value);
        onChange(listener);
    }

    private void update() {
        if (!Logger.hasReplaySource()) {
            value = getValue(entry, defaultValue);
        }
        Logger.processInputs(PREFIX, inputs);

        if (!Objects.equals(value, prevValue)) {
            for (Consumer<T> listener : changeListeners) {
                listener.accept(value);
            }
        }
        prevValue = value;
    }
}
