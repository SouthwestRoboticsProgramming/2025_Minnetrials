package com.swrobotics.robot.pathfinding;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/**
 * An obstacle for the pathfinder to avoid
 */
public abstract class Obstacle {
    abstract void addToJNIObstacleList(long obsHandle);

    /**
     * Loads a set of obstacles from a JSON file.
     *
     * @param fileName path to the file in the deploy directory
     * @return loaded obstacles
     * @throws IOException if the file loading fails
     */
    public static List<Obstacle> loadObstaclesFromJson(String fileName) throws IOException {
        ObjectMapper mapper = new ObjectMapper();

        File file = new File(Filesystem.getDeployDirectory(), fileName);
        List<Obstacle> obstacles = new ArrayList<>();

        JsonNode rootNode = mapper.readTree(file);
        for (Iterator<JsonNode> iter = rootNode.elements(); iter.hasNext();) {
            JsonNode node = iter.next();
            String type = node.get("type").asText();

            Obstacle obs = switch (type) {
                case "Circle" -> mapper.treeToValue(node, Circle.class);
                case "Rectangle" -> mapper.treeToValue(node, Rectangle.class);
                case "Polygon" -> mapper.treeToValue(node, Polygon.class);
                default -> throw new IOException("Unknown obstacle type: " + type);
            };
            obstacles.add(obs);
        }

        return obstacles;
    }
}
