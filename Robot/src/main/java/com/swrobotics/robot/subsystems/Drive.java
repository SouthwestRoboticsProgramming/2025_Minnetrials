package com.swrobotics.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Drive extends SubsystemBase {
    private final TalonSRX leftMotor = new TalonSRX(0); // CAN ID 0
    private final TalonSRX rightMotor = new TalonSRX(1); // CAN ID 1

    public Drive() {
        leftMotor.setInverted(true); // Could be the right motor instead
    }

    public void set(double forwardSpeed, double turnSpeed) {
        System.out.printf("Forward: %f Turn: %f%n", forwardSpeed, turnSpeed);
        double leftSpeed = forwardSpeed - turnSpeed;
        double rightSpeed = forwardSpeed + turnSpeed;

        // Find the maximum possible value of (throttle + turn) along the vector
        // that the joystick is pointing, then desaturate the wheel speeds
        double greaterInput = Math.max(Math.abs(forwardSpeed), Math.abs(turnSpeed));
        double lesserInput = Math.min(Math.abs(forwardSpeed), Math.abs(turnSpeed));
        if (greaterInput == 0.0) { // If no input is reported
            leftSpeed = 0.0;
            leftSpeed = 0.0;
        } else {
            double saturatedInput = (greaterInput + lesserInput) / greaterInput;
            leftSpeed /= saturatedInput;
            rightSpeed /= saturatedInput;
        }

        leftMotor.set(TalonSRXControlMode.PercentOutput, leftSpeed);
        rightMotor.set(TalonSRXControlMode.PercentOutput, rightSpeed);

    }
}
