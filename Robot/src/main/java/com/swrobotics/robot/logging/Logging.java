package com.swrobotics.robot.logging;

import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public final class Logging {
    public enum SimMode {
        SIMULATE,
        REPLAY
    }

    public static void initialize(SimMode simMode) {
        if (RobotBase.isReal()) {
            // TODO: Figure out writing to USB
//            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
        } else if (simMode == SimMode.SIMULATE) {
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            String path = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(path));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(path, "_sim")));
        }

        Logger.start();
    }
}
