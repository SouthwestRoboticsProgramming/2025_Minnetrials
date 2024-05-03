package com.swrobotics.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.swrobotics.robot.commands.PlaySongCommand;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.control.ControlBoard;
import com.swrobotics.robot.logging.FieldView;
import com.swrobotics.robot.subsystems.lights.LightsSubsystem;
import com.swrobotics.robot.subsystems.music.MusicSubsystem;
import com.swrobotics.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import java.io.File;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class RobotContainer {
    // Create dashboard choosers
    private final SendableChooser<Command> autoSelector;
    private final SendableChooser<Double> autoDelaySelector;

    public final ControlBoard controlboard;

    public final PowerDistribution pdp;

    // Mechanical
    public final SwerveDrive drive;

    // Fun
    public final LightsSubsystem lights;
    public final MusicSubsystem music;

    private Command musicCommand;

    public RobotContainer() {
        // Turn off joystick warnings in sim
        DriverStation.silenceJoystickConnectionWarning(RobotBase.isSimulation());

        pdp = new PowerDistribution(IOAllocation.CAN.PDP.id(), PowerDistribution.ModuleType.kRev);

        drive = new SwerveDrive();

        // ControlBoard must be initialized last
        lights = new LightsSubsystem(this);
        music = new MusicSubsystem(this);
        controlboard = new ControlBoard(this);

        // Register Named Commands for Auto
        NamedCommands.registerCommand("Example Named Command", Commands.print("The command ran!"));

        // Create a chooser to select the autonomous
        List<AutoEntry> autos = buildPathPlannerAutos();
        autos.sort(Comparator.comparing(AutoEntry::name, String.CASE_INSENSITIVE_ORDER));
        autoSelector = new SendableChooser<>();
        autoSelector.setDefaultOption("None", Commands.none());
        for (AutoEntry auto : autos)
            autoSelector.addOption(auto.name(), auto.cmd());

        // Create a selector to select delay before running auto
        autoDelaySelector = new SendableChooser<>();
        autoDelaySelector.setDefaultOption("None", 0.0);
        for (int i = 0; i < 10; i++) {
            double time = i / 2.0 + 0.5;
            autoDelaySelector.addOption(time + " seconds", time);
        }

        SmartDashboard.putData("Auto Selector", autoSelector);
        SmartDashboard.putData("Auto Delay", autoDelaySelector);
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
        return autoDelaySelector.getSelected();
    }

    public Command getAutonomousCommand() {
        return autoSelector.getSelected();
    }
}
