package com.swrobotics.lib.pathfinding.async;

import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.swrobotics.lib.pathfinding.PathEnvironment;

import edu.wpi.first.math.geometry.Translation2d;

public interface AsyncPathfinderIO {
    /**
     * Requests that a path should be found with the specified parameters.
     *
     * @param env environment to search within
     * @param start position to start the search from (robot position)
     * @param goal position to find a path to
     */
    void requestPath(PathEnvironment env, Translation2d start, Translation2d goal);

    void updateInputs(Inputs inputs);

    // Not AutoLoggedInputs because path can be null
    final class Inputs implements LoggableInputs {
        private static final Translation2d[] EMPTY = {};

        /** Whether the requested path has been calculated */
        public boolean pathReady = false;

        /** The Bezier vertices of the calculated path */
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
