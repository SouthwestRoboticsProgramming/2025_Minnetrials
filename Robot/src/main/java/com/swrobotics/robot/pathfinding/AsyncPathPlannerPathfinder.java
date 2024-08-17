package com.swrobotics.robot.pathfinding;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinder;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Objects;

public final class AsyncPathPlannerPathfinder implements Pathfinder {
    public static Command pathfindToPose(PathEnvironment env, Pose2d goal, PathConstraints constraints) {
        Command ppCommand = AutoBuilder.pathfindToPose(goal, constraints);
        return Commands.sequence(
                Commands.runOnce(() -> INSTANCE.setEnvironment(env)),
                ppCommand
        );
    }

    private static final AsyncPathPlannerPathfinder INSTANCE = new AsyncPathPlannerPathfinder();

    public static AsyncPathPlannerPathfinder getInstance() {
        return INSTANCE;
    }

    // Group values together so they can be stored as one volatile reference,
    // and therefore changed atomically
    private record CalcParams(PathEnvironment env, Translation2d start, Translation2d goal) {}
    private record CalcResult(CalcParams params, List<Translation2d> path) {}

    private final Object notifier = new Object();
    private volatile CalcParams params = null;
    private volatile CalcResult result = null;

    private PathEnvironment env = null;
    private Translation2d startPosition = null;
    private boolean reportedThisResult = false;

    public AsyncPathPlannerPathfinder() {
        // Start the solver thread
        Thread thread = new Thread(this::runThread);
        thread.setDaemon(true); // Thread stops automatically if robot code stops
        thread.setName("Pathfinder Solver Thread");
        thread.start();
    }

    private void runThread() {
        while (true) {
            // Wait for new parameters notification from main robot loop
            try {
                synchronized (notifier) {
                    notifier.wait();
                }
            } catch (InterruptedException e) {
                // don't care, should never happen
            }

            // Keep trying while parameters differ in case they changed while
            // finding the previous path
            CalcParams params = null;
            while (!Objects.equals(params, this.params)) {
                params = this.params;

                List<Translation2d> path = params.env.findPath(params.start, params.goal);
                result = new CalcResult(params, path);
            }
        }
    }

    @Override
    public boolean isNewPathAvailable() {
        // Only report path calculation result once
        if (reportedThisResult)
            return false;

        CalcResult result = this.result;
        return result != null && Objects.equals(params, result.params);
    }

    @Override
    public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
        // PathPlanner always calls this immediately after isNewPathAvailable()

        reportedThisResult = true;

        CalcResult result = this.result;
        List<Translation2d> bezierPoints = result.path;

        Translation2d[] logPath;
        if (bezierPoints == null) {
            // PathPlanner won't accept impossible result, so give it a straight line as a fallback
            // This can only happen if the target is inside an obstacle, which is pretty
            // easy to avoid doing
            DriverStation.reportWarning("Pathfinding: No path found", false);
            bezierPoints = List.of(result.params.start, result.params.start, result.params.goal, result.params.goal);

            logPath = new Translation2d[0];
        } else {
            logPath = new Translation2d[bezierPoints.size()];
            bezierPoints.toArray(logPath);
        }

        Logger.recordOutput("Pathfinding/Start Position", result.params.start);
        Logger.recordOutput("Pathfinding/Goal Position", result.params.goal);
        Logger.recordOutput("Pathfinding/Path", logPath);

        return new PathPlannerPath(bezierPoints, constraints, goalEndState);
    }

    public void setEnvironment(PathEnvironment env) {
        this.env = env;
    }

    @Override
    public void setStartPosition(Translation2d startPosition) {
        this.startPosition = startPosition;
    }

    @Override
    public void setGoalPosition(Translation2d goalPosition) {
        // PathPlanner always calls this immediately after setStartPosition()

        CalcParams newParams = new CalcParams(env, startPosition, goalPosition);

        // If parameters changed, wake up finding thread
        if (!Objects.equals(newParams, params)) {
            this.params = newParams;
            synchronized (notifier) {
                notifier.notifyAll();
            }
        }

        // PathPlanner expects the path to be reported even if we didn't need
        // to recompute
        reportedThisResult = false;
    }

    @Override
    public void setDynamicObstacles(List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) {
        throw new UnsupportedOperationException("Can't set dynamic obstacles");
    }
}
