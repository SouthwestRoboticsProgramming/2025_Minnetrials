package com.swrobotics.robot.control;

import com.swrobotics.lib.input.XboxController;
<<<<<<< Updated upstream
import com.swrobotics.lib.net.NTBoolean;
import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.lib.net.NTInteger;
import com.swrobotics.lib.utils.MathUtil;
=======
>>>>>>> Stashed changes
import com.swrobotics.robot.RobotContainer;
import com.swrobotics.robot.commands.RumblePatternCommands;
import com.swrobotics.robot.config.Constants;

import edu.wpi.first.wpilibj.DriverStation;
<<<<<<< Updated upstream
import edu.wpi.first.wpilibj.util.Color;
=======
>>>>>>> Stashed changes
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class ControlBoard extends SubsystemBase {
    /**
     * Controls:
     * Left stick: translation
     * Right stick X: rotation
     */

    private final RobotContainer robot;
    public final XboxController controller;

    public ControlBoard(RobotContainer robot) {
        this.robot = robot;

        // Passing deadband here means we don't have to deadband anywhere else
        controller = new XboxController(0, Constants.kDeadband);

        // Endgame Alert
        /** Must restart robot code for the time change to take effect */
        new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Constants.kEndgameAlertTime)
<<<<<<< Updated upstream
        .onTrue(RumblePatternCommands.endgameAlert(driver, 0.75)
                .alongWith(RumblePatternCommands.endgameAlert(operator, 0.75)));

        driver.b.onRising(RumblePatternCommands.endgameAlert(driver, 0.75));

        driveFilter = new DriveAccelFilter(Constants.kDriveControlMaxAccel);

        new Trigger(CHARACTERISE_WHEEL_RADIUS::get).whileTrue(new CharacterizeWheelsCommand(robot.drive));
    }

    private Translation2d getDriveTranslation() {
        double maxSpeed = Constants.kMaxAchievableSpeed;

        Translation2d leftStick = driver.getLeftStick();

        double rawMag = leftStick.getNorm();
        double powerMag = MathUtil.powerWithSign(rawMag, Constants.kDriveControlDrivePower);

        if (rawMag == 0 || powerMag == 0)
            return new Translation2d(0, 0); // No division by 0

        double targetSpeed = powerMag * maxSpeed;
        double filteredSpeed = driveFilter.calculate(targetSpeed);

        return new Translation2d(-leftStick.getY(), -leftStick.getX())
                .div(rawMag) // Normalize translation
                .times(filteredSpeed) // Apply new speed
                .rotateBy(FieldInfo.getAllianceForwardAngle()); // Account for driver's perspective
    }

    private Rotation2d getDriveRotation() {
        double input = MathUtil.powerWithSign(-driver.rightStickX.get(), Constants.kDriveControlTurnPower);
        return Rotation2d.fromRotations(input * Constants.kDriveControlMaxTurnSpeed);
=======
        .onTrue(RumblePatternCommands.endgameAlert(controller, 0.75));
>>>>>>> Stashed changes
    }

    @Override
    public void periodic() {
        if (!DriverStation.isTeleop()) {
            return;
        }
    }
}
