package com.swrobotics.robot.pathfinding;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.pathfinding.Pathfinder;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.List;

// TODO: Do we want to multithread? Depends on pathfinder's performance on RoboRIO
// TODO: Better name for this
public final class PathPlannerPathfinder implements Pathfinder {
    public static Command pathfindToPose(PathEnvironment env, Pose2d goal, PathConstraints constraints) {
        Command ppCommand = AutoBuilder.pathfindToPose(goal, constraints);
        return Commands.sequence(
                Commands.runOnce(() -> INSTANCE.setEnvironment(env)),
                ppCommand
        );
    }

    private static final PathPlannerPathfinder INSTANCE = new PathPlannerPathfinder();

    private PathEnvironment environment;

    private Translation2d startPos, goalPos;
    private boolean paramsChanged;

    public static PathPlannerPathfinder getInstance() {
        return INSTANCE;
    }

    private PathPlannerPathfinder() {
        environment = null;
        paramsChanged = false;
    }

    @Override
    public boolean isNewPathAvailable() {
        if (environment == null)
            DriverStation.reportError("No pathfinding environment set", false);

        // New path is always available - we can compute it in less than one periodic cycle
        return paramsChanged
                && environment != null
                && startPos != null
                && goalPos != null;
    }

    public void setEnvironment(PathEnvironment environment) {
        paramsChanged |= this.environment != environment;
        this.environment = environment;
    }

    @Override
    public void setStartPosition(Translation2d startPos) {
        paramsChanged |= !startPos.equals(this.startPos);
        this.startPos = startPos;
    }

    @Override
    public void setGoalPosition(Translation2d goalPos) {
        paramsChanged |= !goalPos.equals(this.goalPos);
        this.goalPos = goalPos;
    }

    @Override
    public PathPlannerPath getCurrentPath(PathConstraints pathConstraints, GoalEndState goalEndState) {
        paramsChanged = false;

        List<Translation2d> bezierPoints = environment.findPath(startPos, goalPos);
        if (bezierPoints == null) {
            // PathPlanner won't accept impossible result, so give it a straight line as a fallback
            // This can only happen if the target is inside an obstacle, which is pretty
            // easy to avoid doing
            DriverStation.reportWarning("Pathfinding: No path found", false);
            bezierPoints = List.of(startPos, startPos, goalPos, goalPos);
        }

        return new PathPlannerPath(bezierPoints, pathConstraints, goalEndState);
    }

    @Override
    public void setDynamicObstacles(List<Pair<Translation2d, Translation2d>> obstacles, Translation2d robotPos) {
        throw new UnsupportedOperationException("Can't set dynamic obstacles");
    }
}
