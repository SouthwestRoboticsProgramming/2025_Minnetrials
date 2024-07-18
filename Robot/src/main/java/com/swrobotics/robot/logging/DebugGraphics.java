package com.swrobotics.robot.logging;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import java.util.List;

// VERY inefficient way to draw arbitrary lines to a Mechanism2d
public final class DebugGraphics {
    private final Mechanism2d mechanism;
    private int rootIdx;

    public DebugGraphics(String name, double w, double h) {
        mechanism = new Mechanism2d(w, h);
        SmartDashboard.putData(name, mechanism);
        rootIdx = 0;
    }

    public void plotLines(List<Translation2d> positions, Color color) {
        Translation2d first = positions.get(0);
        MechanismRoot2d root = mechanism.getRoot("root" + rootIdx++, first.getX(), first.getY());

        Translation2d prev = first;
        MechanismLigament2d prevLigament = null;
        double prevAngle = 0;
        for (int i = 1; i < positions.size(); i++) {
            Translation2d pos = positions.get(i);

            double dx = pos.getX() - prev.getX();
            double dy = pos.getY() - prev.getY();
            double angle = Math.toDegrees(Math.atan2(dy, dx));
            double length = Math.sqrt(dx * dx + dy * dy);

            MechanismLigament2d ligament2d = new MechanismLigament2d("line" + i, length, angle - prevAngle, 2, new Color8Bit(color));
            if (prevLigament == null)
                root.append(ligament2d);
            else
                prevLigament.append(ligament2d);

            prevLigament = ligament2d;
            prev = pos;
            prevAngle = angle;
        }
    }
}
