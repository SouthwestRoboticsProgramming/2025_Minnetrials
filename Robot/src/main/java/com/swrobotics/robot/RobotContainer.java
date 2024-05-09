package com.swrobotics.robot;

import com.swrobotics.robot.commands.DefaultDriveCommand;
import com.swrobotics.robot.control.ControlBoard;
import com.swrobotics.robot.logging.Logging;
import com.swrobotics.robot.subsystems.Drive;
import com.swrobotics.robot.subsystems.Pincher;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;

public class RobotContainer {
    // Whether to simulate the robot or replay a log file
    public static final Logging.SimMode SIM_MODE = Logging.SimMode.SIMULATE;
//    public static final Logging.SimMode SIM_MODE = Logging.SimMode.REPLAY;

    public final LoggedPowerDistribution pdp;
    public final Drive drive;
    public final Pincher pincher;

    public final ControlBoard controlboard;

    public RobotContainer() {
        // Turn off joystick warnings in sim
        DriverStation.silenceJoystickConnectionWarning(RobotBase.isSimulation());

        pdp = LoggedPowerDistribution.getInstance(63, PowerDistribution.ModuleType.kCTRE); // FIXME: Set ID
        drive = new Drive();
        pincher = new Pincher();

        // ControlBoard must be initialized last
        controlboard = new ControlBoard(this);

        drive.setDefaultCommand(new DefaultDriveCommand(drive, () -> controlboard.getDriveForward(), () -> controlboard.getDriveTurn()));
    }

    public Command getAutonomousCommand() {
        // return autoSelector.get();
        return new InstantCommand();
    }
}
