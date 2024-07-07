package com.swrobotics.robot.pathfinding;

import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.Arrays;

@JsonIgnoreProperties(ignoreUnknown = true)
@JsonAutoDetect(getterVisibility = JsonAutoDetect.Visibility.NONE)
public final class Polygon extends Obstacle {
    private final Translation2d[] vertices;

    @JsonCreator
    public Polygon(
            @JsonProperty(required = true, value = "vertices") Translation2d... vertices) {
        this.vertices = vertices;
    }

    public Translation2d[] getVertices() {
        return vertices;
    }

    @Override
    void addToJNIObstacleList(long obsHandle) {
        double[] coords = new double[vertices.length * 2];
        for (int i = 0; i < vertices.length; i++) {
            Translation2d vertex = vertices[i];
            coords[i * 2] = vertex.getX();
            coords[i * 2 + 1] = vertex.getY();
        }
        PathfindingJNI.addPolygon(obsHandle, coords);
    }

    @Override
    public String toString() {
        return "Polygon{" +
                "vertices=" + Arrays.toString(vertices) +
                '}';
    }
}
