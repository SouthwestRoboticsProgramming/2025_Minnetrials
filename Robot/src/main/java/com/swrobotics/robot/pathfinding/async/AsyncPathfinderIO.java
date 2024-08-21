package com.swrobotics.robot.pathfinding.async;

import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.swrobotics.robot.pathfinding.PathEnvironment;

import edu.wpi.first.math.geometry.Translation2d;

public interface AsyncPathfinderIO {
    void requestPath(PathParams params);

    void updateInputs(Inputs inputs);
    

    record PathParams(PathEnvironment env, Translation2d start, Translation2d goal) {}

    final class Inputs implements LoggableInputs {
        private static final Translation2d[] EMPTY = {};

        public boolean pathReady = false;
        public List<Translation2d> path = null;

        @Override
        public void fromLog(LogTable table) {
            pathReady = table.get("pathReady", false);

            Translation2d[] data = table.get("path", EMPTY);
            path = data.length > 0 ? Arrays.asList(data) : null;
        }

        @Override
        public void toLog(LogTable table) {
            table.put("pathReady", pathReady);

            Translation2d[] data = path != null ? path.toArray(EMPTY) : EMPTY;
            table.put("path", data);
        }
    }
}
