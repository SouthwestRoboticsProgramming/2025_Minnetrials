package com.swrobotics.robot.subsystems.vision;

import java.util.List;

public interface VisionSource {
    List<VisionUpdate> getNewUpdates();
}
