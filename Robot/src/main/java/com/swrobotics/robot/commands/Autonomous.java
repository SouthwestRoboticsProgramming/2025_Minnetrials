package com.swrobotics.robot.commands;

import com.swrobotics.robot.RobotContainer;
import com.swrobotics.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autonomous {
    private static Command setIntake(RobotContainer robot, Intake.State state) {
        return Commands.runOnce(() -> robot.intake.setState(state));
    }

    private static Command setArm(RobotContainer robot, boolean up) {
        return Commands.runOnce(() -> robot.arm.set(up));
    }

    private static Command driveForTime(RobotContainer robot, double forward, double turn, double timeSecs) {
        return Commands.run(() -> robot.drive.move(forward, turn))
                .withTimeout(timeSecs)
                .andThen(Commands.runOnce(() -> robot.drive.move(0, 0)));
    }

    public static Command createAutoSequence(RobotContainer robot) {
        return Commands.sequence(
                // Arm up for first shot
                setArm(robot, true),
                // Drive forward to leave starting zone
                driveForTime(robot, 0.4, 0, 0.9),
                // Shoot preloaded ball
                setIntake(robot, Intake.State.EJECT),
                Commands.waitSeconds(0.5),

                // Arm down to pick up ball from field
                setArm(robot, false),
                // Intake on
                setIntake(robot, Intake.State.INTAKE),
                // Drive forward slowly to pick up ball
                driveForTime(robot, 0.15, 0, 1.7),
                // Stop intake to hold ball in intake
                setIntake(robot, Intake.State.OFF),

                // Arm up for second shot
                setArm(robot, true),
                // Wait for arm to finish going up
                Commands.waitSeconds(1),
                // Shoot second ball
                setIntake(robot, Intake.State.EJECT),

                // Wait for shot
                Commands.waitSeconds(1),

                // Put arm back down for teleop
                setArm(robot, false),
                // Intake off
                setIntake(robot, Intake.State.OFF)
        );
    }
}
