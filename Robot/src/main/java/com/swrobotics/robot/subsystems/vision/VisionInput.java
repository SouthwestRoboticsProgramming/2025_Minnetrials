package com.swrobotics.robot.subsystems.vision;

import java.util.List;

public interface VisionInput {
    List<VisionUpdate> getNewUpdates();
}
