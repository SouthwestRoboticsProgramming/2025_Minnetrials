package com.swrobotics.robot.control;

import com.swrobotics.lib.input.XboxController;
import com.swrobotics.robot.RobotContainer;
import com.swrobotics.robot.commands.RumblePatternCommands;
import com.swrobotics.robot.config.Constants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class ControlBoard extends SubsystemBase {
    /**
     * Control mapping:
     *
     * Driver:
     * nothing!
     *
     * Operator:
     * nothing!
     *
     * TODO: Describe the controls here for easy reference
     */

    private final RobotContainer robot;
    public final XboxController driver;
    public final XboxController operator;

    public ControlBoard(RobotContainer robot) {
        this.robot = robot;

        // Passing deadband here means we don't have to deadband anywhere else
        driver = new XboxController(Constants.kDriverControllerPort, Constants.kDeadband);
        operator = new XboxController(Constants.kOperatorControllerPort, Constants.kDeadband);

        // Endgame Alert
        new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0 // Whether match is actually playing
                    && DriverStation.getMatchTime() <= Constants.kEndgameAlertTime)
        .onTrue(RumblePatternCommands.endgameAlert(driver, 0.75)
                .alongWith(RumblePatternCommands.endgameAlert(operator, 0.75)));
    }

    @Override
    public void periodic() {
        if (!DriverStation.isTeleop()) {
            return;
        }

        /*
         * when pressed forward normally it goes 0.5 of full speed until b is
         * pressed to make it go beeg fast
         * 
         * if b is pressed, go beeg fast
         * if b is not pressed, go 0.5 of full speed
         */

        // give input to controler 
        double driveForward = driver.leftStickY.get();
        double driveTurn = driver.leftStickX.get();

        boolean speedButton = driver.b.isDown();

        
        driveForward = Math.copySign(Math.pow(driveForward, 2), driveForward);
        driveTurn = Math.copySign(Math.pow(driveTurn, 2), driveTurn);

        driveTurn *= .5;
        
        // if (speedButton) {
        //     robot.drive.move(driveForward, driveTurn);
        // }
        // else {
        //     robot.drive.move(driveForward/2, ); 
        // }

        if (!speedButton) {
            driveForward /= 2; // Go half the speed
        }

        robot.drive.move(driveForward, driveTurn);


        // TODO: Put teleop control logic here
    }
}
