package com.swrobotics.robot.subsystems.vision;

import java.util.List;

/**
 * A source of vision updates.
 */
public interface VisionSource {
    /**
     * Gets the vision updates that have been received since the last time this
     * method was called
     *
     * @return new vision updates
     */
    List<VisionUpdate> getNewUpdates();
}
