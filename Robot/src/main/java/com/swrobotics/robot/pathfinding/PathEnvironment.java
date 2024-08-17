package com.swrobotics.robot.pathfinding;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public final class PathEnvironment {
    private final List<Obstacle> obstacles;
    private final long handle;

    public PathEnvironment(List<Obstacle> obstacles, double avoidanceRadius) {
        this.obstacles = obstacles;
        long obs = PathfindingJNI.newObstacleList();
        for (Obstacle obstacle : obstacles) {
            obstacle.addToJNIObstacleList(obs);
        }
        handle = PathfindingJNI.buildEnvironment(obs, avoidanceRadius);
    }

    public List<Translation2d> findPath(Translation2d start, Translation2d goal) {
        double[] result = PathfindingJNI.findPath(handle, start.getX(), start.getY(), goal.getX(), goal.getY());
        if (result == null)
            return null; // Didn't find a path

        List<Translation2d> bezierPoints = new ArrayList<>();
        for (int i = 0; i < result.length; i += 2) {
            double x = result[i];
            double y = result[i + 1];
            bezierPoints.add(new Translation2d(x, y));
        }
        return bezierPoints;
    }

    public PathfindingDebug getDebug() {
        return new PathfindingDebug(obstacles, PathfindingJNI.getDebugData(handle));
    }

    public List<Translation2d> debugFindSafe(Translation2d start) {
        double[] data = PathfindingJNI.debugFindSafe(handle, start.getX(), start.getY());
        List<Translation2d> points = new ArrayList<>();
        for (int i = 0; i < data.length; i += 2) {
            double x = data[i];
            double y = data[i + 1];
            points.add(new Translation2d(x, y));
        }
        return points;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        PathEnvironment that = (PathEnvironment) o;
        return handle == that.handle;
    }

    @Override
    public int hashCode() {
        return Objects.hash(handle);
    }
}
