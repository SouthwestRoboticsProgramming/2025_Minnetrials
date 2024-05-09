package com.swrobotics.robot.commands;

import java.util.function.Supplier;

import com.swrobotics.robot.subsystems.Drive;

import edu.wpi.first.wpilibj2.command.Command;

public class DefaultDriveCommand extends Command {
    private final Drive drive;
    private final Supplier<Double> driveForward, driveTurn;

    public DefaultDriveCommand(Drive drive, Supplier<Double> driveForward, Supplier<Double> driveTurn) {
        this.driveForward = driveForward;
        this.driveTurn = driveTurn;
        this.drive = drive;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.set(driveForward.get(), driveTurn.get());
    }
}
