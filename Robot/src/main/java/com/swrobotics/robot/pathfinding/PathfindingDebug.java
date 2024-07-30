package com.swrobotics.robot.pathfinding;

import com.swrobotics.lib.field.FieldInfo;
import com.swrobotics.lib.utils.MathUtil;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.logging.DebugGraphics;
import com.swrobotics.robot.logging.FieldView;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Color;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public final class PathfindingDebug {
    public static DebugGraphics g = new DebugGraphics("Pathfinding Debug", Constants.kField.getWidth(), Constants.kField.getHeight());


    public static final class Arc {
        public final double centerX, centerY;
        public final double radius;
        public final double minAngle, maxAngle;

        public Arc(double[] data, int i) {
            centerX = data[i];
            centerY = data[i + 1];
            radius = data[i + 2];
            minAngle = data[i + 3];
            maxAngle = data[i + 4];
        }
    }

    public static final class Segment {
        public final double x1, y1;
        public final double x2, y2;

        public Segment(double[] data, int i) {
            x1 = data[i];
            y1 = data[i + 1];
            x2 = data[i + 2];
            y2 = data[i + 3];
        }
    }

    public final List<Obstacle> obstacles;
    public final Arc[] arcs;
    public final Segment[] segments;

    public PathfindingDebug(List<Obstacle> obstacles, double[] data) {
        this.obstacles = obstacles;

        arcs = new Arc[(int) data[0]];
        for (int i = 0; i < arcs.length; i++) {
            arcs[i] = new Arc(data, i * 5 + 1);
        }

        int base = arcs.length * 5 + 1;
        segments = new Segment[(int) data[base]];
        for (int i = 0; i < segments.length; i++) {
            segments[i] = new Segment(data, base + i * 4 + 1);
        }
    }

    public void plot() {
        for (Obstacle obs : obstacles) {
            if (obs instanceof Circle circle) {
                List<Translation2d> points = new ArrayList<>();
                for (int i = 0; i <= 10; i++) {
                    double angle = i / 10.0 * Math.PI * 2;

                    points.add(new Translation2d(
                            circle.getCenter().getX() + circle.getRadius() * Math.cos(angle),
                            circle.getCenter().getY() + circle.getRadius() * Math.sin(angle)
                    ));
                }
                g.plotLines(points, Color.kOrange);
            } else if (obs instanceof Polygon poly) {
                List<Translation2d> points = new ArrayList<>(Arrays.asList(poly.getVertices()));
                points.add(points.get(0));
                g.plotLines(points, Color.kOrange);
            } else if (obs instanceof Rectangle rect) {
                List<Translation2d> points = new ArrayList<>(Arrays.asList(rect.asPolygon().getVertices()));
                points.add(points.get(0));
                g.plotLines(points, Color.kOrange);
            }
        }

        for (Segment seg : segments) {
            g.plotLines(List.of(
                    new Translation2d(seg.x1, seg.y1),
                    new Translation2d(seg.x2, seg.y2)
            ), Color.kYellow);

            Translation2d center = new Translation2d((seg.x1 + seg.x2) / 2, (seg.y1 + seg.y2) / 2);
            Translation2d perp = new Translation2d(seg.y1 - seg.y2, seg.x2 - seg.x1);
            perp = perp.div(perp.getNorm()).times(0.2);

            g.plotLines(List.of(
                    center,
                    center.plus(perp)
            ), Color.kRed);
        }

        for (Arc arc : arcs) {
            double max = arc.maxAngle;
            if (max <= arc.minAngle)
                max += Math.PI * 2;

            List<Translation2d> points = new ArrayList<>();
            for (int i = 0; i <= 5; i++) {
                double f = i / 5.0;
                double angle = MathUtil.lerp(arc.minAngle, max, f);

                points.add(new Translation2d(
                        arc.centerX + arc.radius * Math.cos(angle),
                        arc.centerY + arc.radius * Math.sin(angle)
                ));
            }
            g.plotLines(points, Color.kYellow);
        }
    }
}
