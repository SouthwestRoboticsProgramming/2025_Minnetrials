package com.swrobotics.robot;

import com.swrobotics.lib.ThreadUtils;

import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private final Timer autonomousTimer = new Timer();
    private double autonomousDelay;
    private boolean hasScheduledAuto;

    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        // Create a RobotContainer to manage our subsystems and our buttons
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        Threads.setCurrentThreadPriority(true, 99);

        ThreadUtils.runMainThreadOperations();
        CommandScheduler.getInstance().run(); // Leave this alone
    }

    @Override
    public void autonomousInit() {
        // If an autonomous command has already be set, reset it
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
            System.out.println("Canceled the current auto command");
        }

        // Get autonomous from selector
        autonomousCommand = robotContainer.getAutonomousCommand();

        autonomousDelay = robotContainer.getAutoDelay();
        autonomousTimer.restart();
        hasScheduledAuto = false;

        robotContainer.drive.setEstimatorIgnoreVision(true);
    }

    @Override
    public void autonomousPeriodic() {
        // Manually time auto delay since using sequential group causes crash
        // when running the same auto twice
        if (!hasScheduledAuto && autonomousCommand != null && autonomousTimer.hasElapsed(autonomousDelay)) {
            autonomousCommand.schedule();
            hasScheduledAuto = true;
        }
    }

    @Override
    public void autonomousExit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void disabledInit() {
        robotContainer.disabledInit();
    }

    @Override
    public void disabledExit() {
        robotContainer.disabledExit();
    }

    // Override these so WPILib doesn't print unhelpful warnings
    @Override public void simulationPeriodic() {}
    @Override public void disabledPeriodic() {}
    @Override public void teleopPeriodic() {}
    @Override public void testPeriodic() {}
}
