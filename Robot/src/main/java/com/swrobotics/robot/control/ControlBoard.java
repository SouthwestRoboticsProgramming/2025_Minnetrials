package com.swrobotics.robot.control;

import com.swrobotics.lib.input.XboxController;
import com.swrobotics.robot.RobotContainer;
import com.swrobotics.robot.commands.RumblePatternCommands;
import com.swrobotics.robot.config.Constants;

import com.swrobotics.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class ControlBoard extends SubsystemBase {
    /**
     * Control mapping:
     *
     * Driver:
     * Left stick: forward/backward drive
     * Right stick: turn
     *
     * Operator:
     * A: intake
     * Left trigger: arm up
     * Right trigger: eject
     */

    private final RobotContainer robot;
    public final XboxController driver;
    public final XboxController operator;

    private boolean intaking = false;

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
    
        // Map intake to A button
        new Trigger(operator.a::isDown)
            .onTrue(Commands.runOnce(() -> intaking = true))
            .onFalse(Commands.runOnce(() -> intaking = false));

        // Disable intake when piece detected
        new Trigger(robot.intake::hasPiece)
            .debounce(0.3)
            .onTrue(Commands.runOnce(() -> intaking = false));
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
        double driveForward = -driver.leftStickY.get();
        double driveTurn = driver.rightStickX.get();
        boolean speedButton = driver.b.isDown();

        // boolean intake = operator.a.isDown();
        boolean armUp = operator.leftTrigger.isOutside(Constants.kTriggerThreshold);
        boolean shoot = operator.rightTrigger.isOutside(Constants.kTriggerThreshold);

        //give speed controll based on joysick strength
        driveForward = Math.copySign(Math.pow(driveForward, 2), driveForward);
        driveTurn = Math.copySign(Math.pow(driveTurn, 2), driveTurn);

        driveTurn *= 0.35;
        
        // if (speedButton) {
        //     robot.drive.move(driveForward, driveTurn);
        // }
        // else {
        //     robot.drive.move(driveForward/2, ); 
        // }

        if (!speedButton) {
            driveForward /= 2; // Go half the speed
        }
        
        //robot moves
        robot.drive.move(driveForward, driveTurn);

        //give controler imput
        
        if (intaking) { 
            // intake button pressed
            robot.intake.setState(Intake.State.INTAKE);
//            robot.intake.move(Constants.kIntakeSpeed.get(), Constants.kIntakeSpeed.get());
        } else { 
            // intake button not pressed
            if (shoot) {
                // shoot button pressed
                robot.intake.setState(Intake.State.EJECT);
//                robot.intake.move(-Constants.kIntakeEjectSpeedTop.get(), -Constants.kIntakeEjectSpeedBottom.get());
            } else {
                robot.intake.setState(Intake.State.OFF);
//                robot.intake.move(-Constants.kIntakeIdleSpeed.get(), -Constants.kIntakeIdleSpeed.get());

            }
        } 

                if (armUp) {
                        robot.arm.set(true);
//                    robot.arm.set(Constants.kArmUpAngle.get());
                } else {
                    robot.arm.set(false);
//                    robot.arm.set(Constants.kArmDownAngle.get());
                }

        
        
    }
}
