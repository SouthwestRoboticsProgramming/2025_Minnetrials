package com.swrobotics.robot;

import java.io.File;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.swrobotics.robot.commands.PlaySongCommand;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.control.ControlBoard;
import com.swrobotics.robot.logging.FieldView;
import com.swrobotics.robot.logging.Logging;
import com.swrobotics.robot.subsystems.lights.LightsSubsystem;
import com.swrobotics.robot.subsystems.music.MusicSubsystem;
import com.swrobotics.robot.subsystems.motortracker.MotorTrackerSubsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.subsystems.vision.AprilTagEnvironment;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

/**
 * The container for all of the robot's subsystems. This is separate from
 * {@link Robot} so that we can use a constructor for initialization instead of
 * {@code robotInit()}, which allows us to have final fields for subsystems.
 */
public class RobotContainer {
    // Whether to simulate the robot or replay a log file
    public static final Logging.SimMode SIM_MODE = Logging.SimMode.SIMULATE;
//    public static final Logging.SimMode SIM_MODE = Logging.SimMode.SIMULATE_AND_LOG;
//    public static final Logging.SimMode SIM_MODE = Logging.SimMode.REPLAY;

    // Create dashboard choosers
    private final LoggedDashboardChooser<Command> autoSelector;
    private final LoggedDashboardChooser<Double> autoDelaySelector;

    public final LoggedPowerDistribution pdp;
    public final MotorTrackerSubsystem motorTracker;
    public final LightsSubsystem lights;
    public final MusicSubsystem music;
    // Add more subsystems here

    public final ControlBoard controlboard;

    private Command musicCommand;

    public RobotContainer() {
        // Turn off joystick warnings in sim
        DriverStation.silenceJoystickConnectionWarning(RobotBase.isSimulation());

        // These must be initialized first
        music = new MusicSubsystem();
        motorTracker = new MotorTrackerSubsystem();

        pdp = LoggedPowerDistribution.getInstance(IOAllocation.CAN.PDP.id(), PowerDistribution.ModuleType.kRev);
        lights = new LightsSubsystem(this);

        // ControlBoard must be initialized last
        controlboard = new ControlBoard(this);

        // Register Named Commands for Auto
        NamedCommands.registerCommand("Example Named Command", Commands.print("The command ran!"));

        // Create a chooser to select the autonomous
        autoSelector = new LoggedDashboardChooser<>("Auto Selector");
        autoSelector.addDefaultOption("None", Commands.none());
        autoSelector.addOption("Test", Commands.print("Test worked!"));

//        List<AutoEntry> autos = buildPathPlannerAutos();
//        autos.sort(Comparator.comparing(AutoEntry::name, String.CASE_INSENSITIVE_ORDER));
//        for (AutoEntry auto : autos)
//            autoSelector.addOption(auto.name(), auto.cmd());

        // Create a selector to select delay before running auto
        autoDelaySelector = new LoggedDashboardChooser<>("Auto Delay");
        autoDelaySelector.addDefaultOption("None", 0.0);
        for (int i = 0; i < 10; i++) {
            double time = i / 2.0 + 0.5;
            autoDelaySelector.addOption(time + " seconds", time);
        }

        FieldView.publish();

        // Play startup song
        CommandScheduler.getInstance().schedule(musicCommand = Commands.waitSeconds(5)
                .andThen(new PlaySongCommand(music, "music" + File.separator + "xp.chrp")));
    }

    private static final record AutoEntry(String name, Command cmd) {}

    private static List<AutoEntry> buildPathPlannerAutos() {
        if (!AutoBuilder.isConfigured()) {
            throw new RuntimeException(
                    "AutoBuilder was not configured before attempting to build an auto chooser");
        }

        List<String> autoNames = AutoBuilder.getAllAutoNames();
        autoNames.sort(String.CASE_INSENSITIVE_ORDER);

        List<PathPlannerAuto> options = new ArrayList<>();
        for (String autoName : autoNames) {
            PathPlannerAuto auto = new PathPlannerAuto(autoName);

            options.add(auto);
        }

        List<AutoEntry> entries = new ArrayList<>();
        for (PathPlannerAuto auto : options)
            entries.add(new AutoEntry(auto.getName(), auto));

        return entries;
    }

    public void disabledInit() {
        lights.disabledInit();

        if (DriverStation.isEStopped()) {
            if (musicCommand != null)
                musicCommand.cancel();

            // Play abort sound
            CommandScheduler.getInstance().schedule(musicCommand = new PlaySongCommand(
                    music,
                    "music" + File.separator + "abort.chrp"));
        }
    }

    public void disabledExit() {
        if (musicCommand != null)
            musicCommand.cancel();
    }

    public double getAutoDelay() {
        return autoDelaySelector.get();
    }

    public Command getAutonomousCommand() {
        return autoSelector.get();
    }
}
