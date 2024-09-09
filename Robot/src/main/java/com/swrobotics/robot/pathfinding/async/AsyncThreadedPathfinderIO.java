package com.swrobotics.robot.pathfinding.async;

import java.util.List;
import java.util.Objects;

import com.swrobotics.robot.pathfinding.PathEnvironment;
import edu.wpi.first.math.geometry.Translation2d;

public final class AsyncThreadedPathfinderIO implements AsyncPathfinderIO {
    private record PathParams(PathEnvironment env, Translation2d start, Translation2d goal) {}
    private record CalcResult(PathParams params, List<Translation2d> path) {}
    
    private final Object notifier = new Object();

    private volatile PathParams params = null;
    private volatile CalcResult result = null;

    public AsyncThreadedPathfinderIO() {
        // Start the solver thread
        Thread thread = new Thread(this::runThread);
        thread.setDaemon(true); // Thread stops automatically if robot code stops
        thread.setName("Pathfinder Solver Thread");
        thread.start();
    }

    @Override
    public void requestPath(PathEnvironment env, Translation2d start, Translation2d goal) {
        PathParams newParams = new PathParams(env, start, goal);

        // If parameters changed, wake up finding thread
        if (!Objects.equals(newParams, params)) {
            this.params = newParams;
            synchronized (notifier) {
                notifier.notifyAll();
            }
        }
    }

    @Override
    public void updateInputs(Inputs inputs) {
        // Read result once here so it can't change during this method
        CalcResult result = this.result;

        inputs.pathReady = result != null && Objects.equals(params, result.params);
        if (inputs.pathReady)
            inputs.path = result.path;
    }

    private void runThread() {
        while (true) {
            // Wait for new parameters notification from main robot loop
            try {
                synchronized (notifier) {
                    // Wait with 1 sec timeout in case we somehow miss a notification
                    notifier.wait(1000L);
                }
            } catch (InterruptedException e) {
                // don't care, should never happen
            }

            CalcResult prevResult = result;
            PathParams params = prevResult != null ? prevResult.params : null;

            // Keep trying while parameters differ in case they changed while
            // finding the previous path
            while (!Objects.equals(params, this.params)) {
                params = this.params;

                List<Translation2d> path = params.env().findPath(params.start(), params.goal());
                result = new CalcResult(params, path);
            }
        }
    }
}
