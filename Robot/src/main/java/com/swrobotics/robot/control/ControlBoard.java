package com.swrobotics.robot.control;

import com.swrobotics.lib.input.XboxController;
import com.swrobotics.robot.RobotContainer;
import com.swrobotics.robot.utils.MathUtil;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class ControlBoard extends SubsystemBase {

    public final XboxController controller;

    public ControlBoard(RobotContainer robot) {
        // Passing deadband here means we don't have to deadband anywhere else
        controller = new XboxController(0, 0.2);

        new Trigger(() -> controller.a.isPressed())
            .onTrue(runOnce(() -> robot.pincher.set(true)))
            .onFalse(runOnce(() -> robot.pincher.set(false)));
    }

    public double getDriveForward() {
        return -MathUtil.powerWithSign(controller.getLeftStick().getY(), 2); // Might be positive
    }

    public double getDriveTurn() {
        return -MathUtil.powerWithSign(controller.getLeftStick().getX(), 2);
    }
}
